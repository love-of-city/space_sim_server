// Keep slow reads and mission mutations from publishing contradictory UI state.
export class RendezvousSession {
  constructor() {
    this.busy = false;
    this.revision = 0;
    this.poll = null;
  }

  async refresh(load, render, failed) {
    if (this.busy || this.poll) return;
    const revision = this.revision;
    const controller = new AbortController();
    this.poll = controller;
    const timeout = setTimeout(() => controller.abort(), 10000);
    try {
      const data = await load(controller.signal);
      if (revision === this.revision && !this.busy) render(data);
    } catch (error) {
      if (revision === this.revision && !this.busy) failed(error);
    } finally {
      clearTimeout(timeout);
      if (this.poll === controller) this.poll = null;
    }
  }

  async act(work) {
    if (this.busy) return false;
    this.busy = true;
    this.revision += 1;
    this.poll?.abort();
    try {
      await work();
      return true;
    } finally {
      this.revision += 1;
      this.busy = false;
    }
  }
}

export async function requestJson(path, options = {}, fetcher = globalThis.fetch) {
  let response;
  try {
    response = await fetcher(path, {cache: 'no-store', ...options});
  } catch (error) {
    if (error.name === 'AbortError') throw new Error('状态读取超时，请检查连接');
    throw new Error('无法连接后端，请检查连接');
  }
  if (response.status === 401) throw new Error('请先返回控制台登录');
  let data;
  try { data = await response.json(); }
  catch { throw new Error(`服务返回无效响应（HTTP ${response.status}）`); }
  if (!response.ok) {
    const detail = typeof data?.detail === 'string' ? data.detail : `请求失败（HTTP ${response.status}）`;
    throw new Error(detail);
  }
  if (!data || typeof data !== 'object' || Array.isArray(data)) throw new Error('服务返回无效数据');
  return data;
}
