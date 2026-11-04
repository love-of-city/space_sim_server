// SSH forwards HTTP and WebSocket, but WebRTC still needs ICE/TURN. Reuse the
// platform's authenticated session when signalling is reached over loopback.
export function validateSessionEndpoint(raw, playerHost, origins, jwtSecret) {
  if (!raw) return null;
  const url = new URL(raw);
  if (jwtSecret || playerHost !== "127.0.0.1" || !origins.size ||
      url.protocol !== "http:" || url.hostname !== "127.0.0.1" ||
      url.pathname !== "/api/client-config" || url.username || url.password || url.search || url.hash) {
    throw new Error("Session signalling requires loopback binding, a loopback client-config endpoint, explicit origins and no JWT mode");
  }
  return url;
}

export async function sessionAccess(endpoint, cookie) {
  if (!cookie) throw new Error("Session required");
  const response = await fetch(endpoint, {
    headers: { Cookie: cookie }, redirect: "error", signal: AbortSignal.timeout(5000),
  });
  if (!response.ok) throw new Error("Session rejected");
  const config = await response.json();
  if (!Array.isArray(config.pixel_streaming_streamers) || !config.pixel_streaming_streamers.length) {
    throw new Error("No authorized streams");
  }
  if (config.pixel_streaming_streamers.some(item => !item || typeof item !== "object" ||
      (item.aliases !== undefined && !Array.isArray(item.aliases)))) throw new Error("Invalid stream permissions");
  const ids = config.pixel_streaming_streamers.flatMap(item => [item.id, ...(item.aliases || [])]);
  if (ids.some(id => typeof id !== "string" || !id)) throw new Error("Invalid stream permissions");
  return { streamer_ids: [...new Set(ids)] };
}
