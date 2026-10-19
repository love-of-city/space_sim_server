import template from './users.html?raw';
import {mountView, element, roleLabel, dateTime, askDialog} from '../ui.js';

export function createUsersPage(root, {api, getUser, onUserChange}) {
  const $ = mountView(root, template);
  let users = [], editing = null, epoch = 0, active = false, saving = false;
  function notice(message = '', error = false) { $('notice').textContent = message; $('notice').hidden = !message; $('notice').className = `notice ${error ? 'error' : 'success'}`; }
  function closeEditor() { $('editor').hidden = true; $('user-form').reset(); $('password').value = ''; editing = null; }
  function roleDescription() { $('role-description').textContent = $('role').value === 'admin' ? '管理所有用户、采集批次和数据记录；可管理共享场景。' : '使用仿真工作台与自动采集；管理自己的场景、采集批次和数据记录。'; }
  function openEditor(user = null) {
    closeEditor(); editing = user;
    $('editor-title').textContent = user ? `编辑用户 · ${user.username}` : '新建用户';
    $('username').value = user?.username || ''; $('username').readOnly = Boolean(user);
    $('display-name').value = user?.display_name || user?.username || '';
    $('role').value = user?.role || 'operator'; $('active').value = String(user?.active ?? true);
    const self = user?.user_id === getUser()?.user_id;
    $('role').disabled = self; $('active').disabled = self; $('self-note').hidden = !self;
    $('password-field').hidden = Boolean(user); $('password').required = !user;
    $('form-error').hidden = true; $('save-user').textContent = user ? '保存修改' : '创建用户'; $('save-user').disabled = false;
    $('editor').hidden = false; roleDescription(); (user ? $('display-name') : $('username')).focus();
  }
  function render() {
    const query = $('search').value.trim().toLowerCase();
    const matches = users.filter(user => `${user.username} ${user.display_name || ''}`.toLowerCase().includes(query)
      && ($('filter-role').value === 'all' || user.role === $('filter-role').value)
      && ($('filter-status').value === 'all' || String(user.active) === $('filter-status').value));
    $('rows').replaceChildren(...matches.map(user => {
      const row = element('tr'), identity = element('td', 'user-id');
      identity.append(element('strong', '', user.username), element('small', '', `${user.display_name || user.username}${user.user_id === getUser()?.user_id ? ' · 当前账号' : ''}`));
      const role = element('td'); role.append(element('span', `badge ${user.role === 'admin' ? '' : 'neutral'}`, roleLabel(user.role)));
      const status = element('td'); status.append(element('span', `badge ${user.active ? 'success' : 'neutral'}`, user.active ? '启用' : '停用'));
      const actions = element('td'), group = element('div', 'actions');
      for (const [label, handler] of [['编辑', () => openEditor(user)], [user.active ? '停用' : '启用', () => toggle(user)], ['重置密码', () => resetPassword(user)]]) {
        const button = element('button', 'button text-button', label); button.type = 'button'; button.setAttribute('aria-label', `${label} ${user.username}`);
        button.disabled = label !== '编辑' && user.user_id === getUser()?.user_id;
        button.onclick = () => { void handler(); }; group.append(button);
      }
      actions.append(group); row.append(identity, role, status, element('td', '', dateTime(user.last_login_at_ns)), actions); return row;
    }));
    $('count').textContent = `${matches.length} / ${users.length} 个用户`; $('empty').hidden = matches.length > 0;
  }
  async function refresh() {
    const request = ++epoch; $('refresh').disabled = true;
    try { const result = await api('/api/users'); if (!active || request !== epoch) return; users = result.users; render(); }
    catch (error) { if (active && request === epoch) notice(error.message, true); }
    finally { if (request === epoch) $('refresh').disabled = false; }
  }
  async function toggle(user) {
    const version = epoch;
    const result = await askDialog({title:`${user.active ? '停用' : '启用'}账号 ${user.username}`, message:user.active ? '该账号将无法登录，现有登录会话会被撤销。已有数据保留，后台采集不会因此自动停止。' : '账号将恢复登录，并按其角色访问平台。', confirmLabel:user.active ? '确认停用' : '确认启用', destructive:user.active});
    if (!result || !active || version !== epoch) return;
    try { await api(`/api/users/${encodeURIComponent(user.user_id)}`, {method:'PATCH', body:JSON.stringify({display_name:user.display_name || user.username, role:user.role, active:!user.active})}); if (!active || version !== epoch) return; notice(`已${user.active ? '停用' : '启用'} ${user.username}`); await refresh(); }
    catch (error) { if (active && version === epoch) notice(error.message, true); }
  }
  async function resetPassword(user) {
    const version = epoch;
    const result = await askDialog({title:`重置 ${user.username} 的密码`, message:'重置后，该账号的现有登录会话会失效。', fields:[{name:'password', label:'新密码', type:'password', minLength:8, autocomplete:'new-password'}], confirmLabel:'重置密码'});
    if (!result || !active || version !== epoch) return;
    try { await api(`/api/users/${encodeURIComponent(user.user_id)}/reset-password`, {method:'POST', body:JSON.stringify(result)}); if (active && version === epoch) notice(`已重置 ${user.username} 的密码`); }
    catch (error) { if (active && version === epoch) notice(error.message, true); }
  }
  $('user-form').onsubmit = async event => {
    event.preventDefault(); if (saving) return;
    const user = editing, version = epoch;
    const values = {display_name:$('display-name').value.trim(), role:$('role').value, active:$('active').value === 'true'};
    if (!user) Object.assign(values, {username:$('username').value.trim(), password:$('password').value});
    saving = true; $('save-user').disabled = true; $('form-error').hidden = true;
    try {
      const saved = await api(user ? `/api/users/${encodeURIComponent(user.user_id)}` : '/api/users', {method:user ? 'PATCH' : 'POST', body:JSON.stringify(values)});
      if (!active || version !== epoch) return;
      if (saved.user_id === getUser()?.user_id) onUserChange(saved);
      closeEditor(); notice(`已${user ? '更新' : '创建'}用户 ${saved.username}`); await refresh();
    } catch (error) { if (active && version === epoch) { $('form-error').textContent = error.message; $('form-error').hidden = false; } }
    finally { saving = false; $('save-user').disabled = false; $('password').value = ''; }
  };
  $('new-user').onclick = () => openEditor(); $('close-editor').onclick = closeEditor; $('cancel-edit').onclick = closeEditor;
  $('role').onchange = roleDescription; $('refresh').onclick = refresh;
  $('search').oninput = render; $('filter-role').onchange = render; $('filter-status').onchange = render;
  return {
    async activate() { active = true; await refresh(); },
    deactivate() { active = false; epoch++; closeEditor(); },
    reset() { active = false; epoch++; users = []; closeEditor(); $('search').value = ''; $('filter-role').value = 'all'; $('filter-status').value = 'all'; notice(); render(); },
  };
}
