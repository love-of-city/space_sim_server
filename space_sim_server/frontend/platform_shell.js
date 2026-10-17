export const routes = { '/': 'operate', '/collection': 'collection', '/datasets': 'datasets', '/users': 'users' };
const titles = { operate:'仿真工作台', collection:'自动采集', datasets:'数据集', users:'用户管理' };

export function routeForUrl(url) {
  const parsed = new URL(url, 'http://platform.local');
  return { page: routes[parsed.pathname.replace(/\/$/, '') || '/'] || 'operate', params: parsed.searchParams };
}

export function navigationUrl(target, current) {
  const url = new URL(target, current);
  const previous = new URL(current);
  if (url.origin !== previous.origin || !Object.hasOwn(routes, url.pathname)) throw new Error('无效的平台页面');
  if (!url.searchParams.has('access_key') && previous.searchParams.has('access_key')) url.searchParams.set('access_key', previous.searchParams.get('access_key'));
  return url.pathname + url.search + url.hash;
}

export function createPlatformShell({ onNavigate }) {
  let user = null;
  const nav = document.getElementById('platformNav');
  const layout = document.getElementById('platformLayout');
  const collapse = document.getElementById('collapseNavigation');
  function setUser(value) {
    user = value;
    document.querySelectorAll('[data-admin-only]').forEach(node => { node.hidden = value?.role !== 'admin'; });
  }
  async function activateCurrent() {
    if (!user) return;
    let route = routeForUrl(location.href);
    if (route.page === 'users' && user.role !== 'admin') {
      history.replaceState(null, '', navigationUrl('/', location.href));
      route = routeForUrl(location.href);
    }
    nav.querySelectorAll('[data-route]').forEach(link => {
      if (routeForUrl(link.href).page === route.page) link.setAttribute('aria-current', 'page');
      else link.removeAttribute('aria-current');
    });
    document.getElementById('platformPageTitle').textContent = titles[route.page];
    document.title = `${titles[route.page]} · Space Arm`;
    await onNavigate(route.page, route.params);
  }
  function navigate(target, {replace = false} = {}) {
    const url = navigationUrl(target, location.href);
    if (replace) history.replaceState(null, '', url); else if (url !== location.pathname + location.search + location.hash) history.pushState(null, '', url);
    return activateCurrent();
  }
  document.addEventListener('click', event => {
    const link = event.target.closest('a[data-route]');
    if (!link || event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey || link.target === '_blank') return;
    if (link.origin !== location.origin) return;
    event.preventDefault(); void navigate(link.href);
  });
  window.addEventListener('popstate', () => { void activateCurrent(); });
  collapse.onclick = () => {
    const collapsed = layout.classList.toggle('nav-collapsed');
    collapse.setAttribute('aria-expanded', String(!collapsed));
    collapse.setAttribute('aria-label', collapsed ? '展开导航' : '收起导航');
  };
  return { navigate, activateCurrent, setUser };
}
