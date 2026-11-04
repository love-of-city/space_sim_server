// Authenticated HTTP requests and consistent API error decoding.
export function createApiClient({ state, shutdownAuthenticatedApp, showLogin, fetch = globalThis.fetch }) {
  async function apiRequest(url, options = {}) {
    let response;
    try {
      response = await fetch(url, {...options, signal: options.signal ?? globalThis.AbortSignal?.timeout(10000)});
    } catch (error) {
      if (error.name === 'AbortError' || error.name === 'TimeoutError') throw new Error('请求超时，请检查连接');
      throw new Error('无法连接后端，请检查连接');
    }
    if (response.status === 401 && url !== "/api/auth/login") {
      const error = await response.clone().json().catch(() => ({}));
      if (error.detail === "a valid stream access key is required" || !state.currentUser) return response;
      shutdownAuthenticatedApp();
      showLogin("登录已过期，请重新登录");
    }
    return response;
  }

  async function readApiResponse(response) {
    const text = await response.text();
    if (!text) return {};
    try { return JSON.parse(text); }
    catch (_) {
      if (response.ok) throw new Error(`服务返回无效响应（HTTP ${response.status}）`);
      return { detail: text };
    }
  }

  async function platformApi(url,options={}) {
   const response=await apiRequest(url,{...options,cache:'no-store',headers:{...(options.body?{'Content-Type':'application/json'}:{}),...options.headers}});
   const data=await readApiResponse(response);
   if(!response.ok){const detail=data.detail;throw new Error(typeof detail==='string'?detail:Array.isArray(detail)?detail.map(item=>item.msg).join('；'):'请求失败，请稍后重试');}
   return data;
  }

  return { apiRequest, readApiResponse, platformApi };

}
