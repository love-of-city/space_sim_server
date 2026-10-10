/** Build the viewer URL from this platform's actual streaming configuration. */
export function rendezvousPlayerUrl(pageUrl, config) {
  const port = Number(config.pixel_streaming_player_port);
  const streamer = config.pixel_streaming_streamer_id;
  if (!Number.isInteger(port) || port < 1 || port > 65535 || typeof streamer !== 'string' || !streamer) {
    throw new Error('播放器配置无效');
  }
  const player = new URL(pageUrl);
  player.port = String(port);
  player.pathname = '/player.html';
  player.search = new URLSearchParams({ StreamerId: streamer, AutoConnect: 'true', AutoPlayVideo: 'true' }).toString();
  player.hash = '';
  return player.href;
}
