/** Observe authorized camera availability without subscribing to a media peer.
 * The UE 5.6 SDK recreates its peer on streamer-list refresh, even for the same
 * selection. A separate discovery socket keeps that refresh away from playback.
 */
export function observePreviewAvailability(url, onIds, {
  Socket = WebSocket, schedule = setTimeout, cancel = clearTimeout,
} = {}) {
  let current, retry, closed = false;
  function connect() {
    if (closed) return;
    const socket = new Socket(url);
    current = socket;
    socket.addEventListener('open', () => {
      if (!closed && current === socket) socket.send(JSON.stringify({type: 'listStreamers'}));
    });
    socket.addEventListener('message', event => {
      if (closed || current !== socket) return;
      let message;
      try { message = JSON.parse(event.data); } catch { return; }
      if (message.type === 'streamerList' && Array.isArray(message.ids) && message.ids.every(id => typeof id === 'string')) {
        onIds([...new Set(message.ids)]);
      }
    });
    socket.addEventListener('close', () => {
      if (closed || current !== socket) return;
      current = null;
      retry = schedule(() => { retry = null; connect(); }, 1500);
    });
    socket.addEventListener('error', () => socket.close());
  }
  connect();
  return {close() {
    closed = true;
    if (retry != null) cancel(retry);
    retry = null;
    current?.close();
    current = null;
  }};
}
