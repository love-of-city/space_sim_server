import crypto from "node:crypto";
import { PeerDeparture } from "./peer_departure.mjs";
import { parseAllowedOrigins, isAllowedOrigin } from "./origin.mjs";
import { URL } from "node:url";
import WebSocket, { WebSocketServer } from "ws";
import {
  MessageHelpers,
  Messages,
  Logger,
  LogLevel,
  SignallingProtocol,
  WebSocketTransportNJS,
} from "@epicgames-ps/lib-pixelstreamingcommon-ue5.6";
import {
  PlayerRegistry,
  StreamerConnection,
  StreamerRegistry,
} from "@epicgames-ps/lib-pixelstreamingsignalling-ue5.6";

const playerPort = Number(process.env.PS_PLAYER_PORT || 8080);
const streamerPort = Number(process.env.PS_STREAMER_PORT || 8888);
const playerHost = process.env.PS_PLAYER_HOST || "0.0.0.0";
const streamerHost = process.env.PS_STREAMER_HOST || "127.0.0.1";
const maxSubscribers = Number(process.env.PS_MAX_SUBSCRIBERS || 4);
const jwtSecret = process.env.PS_JWT_SECRET || "";
const allowedOrigins = parseAllowedOrigins(process.env.PS_ALLOWED_ORIGINS);
const staticIceServers = parseJsonArray(process.env.PS_ICE_SERVERS_JSON);
const turnUrls = parseJsonArray(process.env.PS_TURN_URLS_JSON).map(String);
const turnSecret = process.env.PS_TURN_AUTH_SECRET || "";
const turnPrefix = process.env.PS_TURN_USERNAME_PREFIX || "space-arm";
const turnTtl = Number(process.env.PS_TURN_CREDENTIAL_TTL_SECONDS || 86400);

if (!jwtSecret) throw new Error("PS_JWT_SECRET is required for secure signalling");
Logger.InitLogging(LogLevel.Info, false);

function parseJsonArray(raw) {
  if (!raw) return [];
  const parsed = JSON.parse(raw);
  if (!Array.isArray(parsed)) throw new Error("ICE/TURN configuration must be a JSON array");
  return parsed;
}

function base64UrlDecode(value) {
  return Buffer.from(value.replace(/-/g, "+").replace(/_/g, "/"), "base64");
}

function verifyAccessToken(token) {
  const parts = String(token || "").split(".");
  if (parts.length !== 3) throw new Error("malformed JWT");
  const signed = `${parts[0]}.${parts[1]}`;
  const expected = crypto.createHmac("sha256", jwtSecret).update(signed).digest();
  const actual = base64UrlDecode(parts[2]);
  if (actual.length !== expected.length || !crypto.timingSafeEqual(actual, expected)) {
    throw new Error("invalid JWT signature");
  }
  const header = JSON.parse(base64UrlDecode(parts[0]).toString("utf8"));
  const payload = JSON.parse(base64UrlDecode(parts[1]).toString("utf8"));
  if (header.alg !== "HS256" || header.typ !== "JWT") throw new Error("unsupported JWT header");
  if (!Number.isFinite(payload.exp) || payload.exp <= Math.floor(Date.now() / 1000)) {
    throw new Error("expired JWT");
  }
  if (!Array.isArray(payload.streamer_ids) || payload.streamer_ids.some((id) => typeof id !== "string")) {
    throw new Error("JWT has no streamer permission list");
  }
  return payload;
}

function peerConnectionOptions() {
  const iceServers = [...staticIceServers];
  if (turnUrls.length && turnSecret) {
    const expiresAt = Math.floor(Date.now() / 1000) + turnTtl;
    const username = `${expiresAt}:${turnPrefix}`;
    const credential = crypto.createHmac("sha1", turnSecret).update(username).digest("base64");
    iceServers.push({ urls: turnUrls, username, credential });
  }
  return { iceServers };
}

function configurationMessage() {
  const message = MessageHelpers.createMessage(Messages.config, {
    protocolVersion: SignallingProtocol.SIGNALLING_VERSION,
  });
  // UE 5.6's generated peerConnectionOptions schema has no declared fields.
  // mergePartial() inside createMessage() drops iceServers. Like Epic's own
  // SignallingServer, attach this extensible RTCConfiguration AFTER creation.
  message.peerConnectionOptions = peerConnectionOptions();
  return message;
}

const streamerRegistry = new StreamerRegistry();
const playerRegistry = new PlayerRegistry();
const context = { streamerRegistry, playerRegistry };

class SafeStreamerConnection extends StreamerConnection {
  constructor(...args) {
    super(...args);
    this.peerDepartures = new Map();
    this.on("disconnect", () => {
      for (const peer of this.peerDepartures.values()) peer.finish(false);
    });
  }
  forwardMessage(message) {
    const playerId = message.playerId;
    if (message.type === Messages.offer.typeName) this.peerDepartures.get(playerId)?.offered();
    // Never deliver late SDP/ICE to a player that switched to another camera.
    const player = playerRegistry.get(playerId);
    if (!player || player.closed || player.subscribedStreamer !== this) return;
    super.forwardMessage(message);
  }
}

function notifyPlayersOfStreamerList() {
  for (const player of playerRegistry.listPlayers()) {
    // The Epic frontend SDK may re-apply its selected streamer whenever a new
    // list arrives.  Do not disturb an established WebRTC session merely
    // because an additional manifest camera registered; only wake players
    // that are still waiting for their requested streamer.
    if (player.subscribedStreamer) continue;
    try { player.listStreamers(); } catch { /* best-effort availability update */ }
  }
}

class AuthorizedPlayerConnection {
  constructor(ws, access, remoteAddress) {
    this.playerId = "";
    this.access = access;
    this.remoteAddress = remoteAddress;
    this.subscribedStreamer = null;
    this.closed = false;
    this.transport = new WebSocketTransportNJS(ws);
    this.protocol = new SignallingProtocol(this.transport);
    this.streamerIdChanged = (newId) => this.send(
      MessageHelpers.createMessage(Messages.streamerIdChanged, { newID: newId })
    );
    this.streamerDisconnected = () => this.disconnect();
    this.transport.on("close", () => this.disconnect());
    this.protocol.on(Messages.subscribe.typeName, (message) => this.subscribe(message.streamerId));
    this.protocol.on(Messages.unsubscribe.typeName, () => this.unsubscribe());
    this.protocol.on(Messages.listStreamers.typeName, () => this.listStreamers());
    this.protocol.on(Messages.ping.typeName, (message) => this.send(
      MessageHelpers.createMessage(Messages.pong, { time: message.time })
    ));
    for (const type of [
      Messages.offer, Messages.answer, Messages.iceCandidate, Messages.dataChannelRequest,
      Messages.peerDataChannelsReady, Messages.layerPreference,
    ]) this.protocol.on(type.typeName, (message) => this.forward(message));
  }

  send(message) { this.protocol.sendMessage(message); }
  getReadableIdentifier() { return this.playerId; }
  getPlayerInfo() {
    return {
      playerId: this.playerId,
      type: "Player",
      subscribedTo: this.subscribedStreamer?.streamerId,
      remoteAddress: this.remoteAddress,
    };
  }
  visibleStreamerIds() {
    return this.access.streamer_ids.filter((id) => streamerRegistry.find(id)?.streaming);
  }
  listStreamers() {
    this.send(MessageHelpers.createMessage(Messages.streamerList, { ids: this.visibleStreamerIds() }));
  }
  subscribe(streamerId) {
    if (this.closed) return;
    // The frontend may repeat its selection on streamer-list refresh. This
    // must not send a rapid playerDisconnected/playerConnected pair to UE.
    if (this.subscribedStreamer?.streamerId === streamerId) return;
    if (!this.access.streamer_ids.includes(streamerId)) {
      this.send(MessageHelpers.createMessage(Messages.subscribeFailed, { message: "Streamer is not permitted by this token." }));
      return;
    }
    const streamer = streamerRegistry.find(streamerId);
    if (!streamer?.streaming) {
      this.send(MessageHelpers.createMessage(Messages.subscribeFailed, { message: "Streamer is not available." }));
      return;
    }
    if (streamer.subscribers.has(this.playerId)) {
      this.send(MessageHelpers.createMessage(Messages.subscribeFailed, { message: "Previous subscription is still closing; reconnect to retry." }));
      return;
    }
    if (streamer.maxSubscribers > 0 && streamer.subscribers.size >= streamer.maxSubscribers) {
      this.send(MessageHelpers.createMessage(Messages.subscribeFailed, { message: "Streamer subscriber limit reached." }));
      return;
    }
    this.unsubscribe();
    this.subscribedStreamer = streamer;
    streamer.subscribers.add(this.playerId);
    const playerId = this.playerId;
    const peer = new PeerDeparture((notify) => {
      streamer.peerDepartures.delete(playerId);
      streamer.subscribers.delete(playerId);
      if (notify) {
        try { streamer.protocol.sendMessage(MessageHelpers.createMessage(Messages.playerDisconnected, { playerId })); }
        catch { /* streamer transport already closed */ }
      }
    });
    streamer.peerDepartures.set(playerId, peer);
    streamer.on("id_changed", this.streamerIdChanged);
    streamer.on("disconnect", this.streamerDisconnected);
    streamer.protocol.sendMessage(MessageHelpers.createMessage(Messages.playerConnected, {
      playerId: this.playerId, dataChannel: true, sfu: false,
    }));
  }
  unsubscribe() {
    if (!this.subscribedStreamer) return;
    const streamer = this.subscribedStreamer;
    // End routing/control immediately. Only UE peer teardown is delayed; this
    // does not keep a disconnected browser authorized or its input active.
    this.subscribedStreamer = null;
    streamer.off("id_changed", this.streamerIdChanged);
    streamer.off("disconnect", this.streamerDisconnected);
    streamer.peerDepartures.get(this.playerId)?.leave();
  }
  forward(message) {
    // Late ICE/answers must never auto-subscribe an unrelated fallback stream.
    if (this.closed || !this.subscribedStreamer) return;
    message.playerId = this.playerId;
    this.subscribedStreamer.protocol.sendMessage(message);
  }
  disconnect() {
    if (this.closed) return;
    this.closed = true;
    this.unsubscribe();
    try { this.protocol.disconnect(); } catch { /* already closed */ }
  }
}

const streamerServer = new WebSocketServer({ host: streamerHost, port: streamerPort, backlog: 16 });
streamerServer.on("connection", (ws, request) => {
  const streamer = new SafeStreamerConnection(context, ws, request.socket.remoteAddress);
  streamer.maxSubscribers = maxSubscribers;
  streamerRegistry.add(streamer);
  streamer.on("id_changed", notifyPlayersOfStreamerList);
  streamer.transport.on("close", () => {
    streamerRegistry.remove(streamer);
    notifyPlayersOfStreamerList();
  });
  streamer.sendMessage(configurationMessage());
});

const playerServer = new WebSocketServer({
  host: playerHost, port: playerPort, backlog: 64,
  // Reject cross-origin browser upgrades before establishing a player session.
  verifyClient: (info, done) => done(isAllowedOrigin(info.origin, allowedOrigins), 403, "Origin not allowed"),
});
playerServer.on("connection", (ws, request) => {
  try {
    const requestUrl = new URL(request.url || "/", "http://localhost");
    const token = requestUrl.searchParams.get("token") || request.headers.authorization?.replace(/^Bearer\s+/i, "");
    const access = verifyAccessToken(token);
    const player = new AuthorizedPlayerConnection(ws, access, request.socket.remoteAddress);
    playerRegistry.add(player);
    player.transport.on("close", () => playerRegistry.remove(player));
    player.send(configurationMessage());
  } catch (error) {
    ws.close(4401, error instanceof Error ? error.message : "Unauthorized");
  }
});

console.log(JSON.stringify({ event: "ready", playerHost, playerPort, streamerHost, streamerPort, secure: true }));
const close = async () => {
  await Promise.all([
    new Promise((resolve) => playerServer.close(resolve)),
    new Promise((resolve) => streamerServer.close(resolve)),
  ]);
  process.exit(0);
};
process.on("SIGINT", close);
process.on("SIGTERM", close);
