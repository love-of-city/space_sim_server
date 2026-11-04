// Read-only stereo diagnostics inside the existing operator console.
export function stereoStatusText(data) {
  if (!data || data.stale) return '等待有效双目图像；旧结果已失效';
  if (data.last_error) return `双目解算失败：${data.last_error}`;
  return JSON.stringify(data.last_result ?? {status: '等待双目解算'}, null, 2);
}

export function createStereoVisionPanel({button, panel, status, images, request, visible,
  schedule = setTimeout, cancel = clearTimeout, urls = URL}) {
  let open = false, generation = 0, timer = null;
  const sources = new Map();
  function clearImages() {
    for (const url of sources.values()) urls.revokeObjectURL(url);
    sources.clear();
    for (const image of Object.values(images)) image.removeAttribute('src');
  }
  async function poll(epoch) {
    if (!open || epoch !== generation) return;
    let failed = false;
    try {
      if (!visible()) { clearImages(); status.textContent = '等待场景就绪'; return; }
      const response = await request('/api/vision/status', {cache: 'no-store', signal: AbortSignal.timeout(1500)});
      if (!response.ok) throw new Error('双目状态暂不可用');
      const data = await response.json();
      if (!open || epoch !== generation) return;
      if (!visible()) { clearImages(); status.textContent = '等待场景就绪'; return; }
      status.textContent = stereoStatusText(data);
      if (data.stale) { clearImages(); return; }
      await Promise.all(Object.entries(images).map(async ([eye, image]) => {
        const response = await request(`/api/vision/image/${eye}`, {cache: 'no-store', signal: AbortSignal.timeout(1500)});
        if (!response.ok) throw new Error('双目图像暂不可用');
        const blob = await response.blob();
        if (!open || epoch !== generation || !visible() || failed) return;
        const previous = sources.get(eye), url = urls.createObjectURL(blob);
        sources.set(eye, url); image.src = url;
        if (previous) urls.revokeObjectURL(previous);
      }));
    } catch (error) {
      failed = true;
      if (open && epoch === generation) { clearImages(); status.textContent = error.message; }
    } finally {
      if (open && epoch === generation) timer = schedule(() => poll(epoch), 1000);
    }
  }
  function setOpen(value) {
    open = value; generation++;
    if (timer !== null) cancel(timer);
    timer = null; panel.hidden = !open; button.setAttribute('aria-expanded', String(open));
    clearImages(); status.textContent = '等待双目图像';
    if (open) void poll(generation);
  }
  button.addEventListener('click', () => setOpen(!open));
  return {setOpen};
}
