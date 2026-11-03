export async function requestJson(path, options = {}, fetcher = globalThis.fetch) {
  let response;
  try {
    response = await fetcher(path, {cache: 'no-store', ...options});
  } catch (error) {
    if (error.name === 'AbortError') throw new Error('状态读取超时，请检查连接');
    throw new Error('无法连接后端，请检查连接');
  }
  if (response.status === 401) throw new Error('登录已过期，请重新登录');
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
