// Shared presentation helpers. Static templates are trusted build inputs;
// all server-provided values are assigned through textContent or form values.
export function mountView(root, template) {
  root.innerHTML = template;
  const nodes = new Map();
  root.querySelectorAll('[id]').forEach(node => {
    const id = node.id;
    nodes.set(id, node);
    node.dataset.field = id;
    node.id = `${root.id}-${id}`;
  });
  root.querySelectorAll('[for], [aria-labelledby], [aria-controls], [aria-describedby]').forEach(node => {
    for (const name of ['for', 'aria-labelledby', 'aria-controls', 'aria-describedby']) {
      if (node.hasAttribute(name)) node.setAttribute(name, node.getAttribute(name).split(' ').map(id => nodes.get(id)?.id || id).join(' '));
    }
  });
  return id => nodes.get(id);
}

export function element(tag, className = '', text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

export function userLabel(user) { return user.display_name || user.username; }
export function roleLabel(role) { return role === 'admin' ? '管理员' : '操作员'; }
export function dateTime(ns) {
  const date = new Date(Number(ns) / 1e6);
  return ns && Number.isFinite(date.getTime()) ? date.toLocaleString('zh-CN', {hour12:false}) : '尚无记录';
}

export function askDialog({ title, message = '', fields = [], confirmLabel = '确认', destructive = false }) {
  return new Promise(resolve => {
    const dialog = element('dialog', 'platform-dialog');
    const form = element('form');
    const heading = element('h2', '', title);
    heading.id = 'platform-dialog-heading';
    dialog.setAttribute('aria-labelledby', heading.id);
    form.append(heading);
    if (message) form.append(element('p', 'muted', message));
    for (const field of fields) {
      const label = element('label', 'field', field.label);
      const input = element('input');
      input.name = field.name;
      input.type = field.type || 'text';
      input.required = field.required !== false;
      if (field.minLength) input.minLength = field.minLength;
      input.maxLength = field.maxLength || 256;
      input.autocomplete = field.autocomplete || 'off';
      label.append(input); form.append(label);
    }
    const actions = element('div', 'form-actions');
    const cancel = element('button', 'button', '取消'); cancel.type = 'button';
    const confirm = element('button', `button ${destructive ? 'danger' : 'primary'}`, confirmLabel); confirm.type = 'submit';
    actions.append(cancel, confirm); form.append(actions); dialog.append(form); document.body.append(dialog);
    let result = null;
    cancel.onclick = () => dialog.close();
    form.onsubmit = event => { event.preventDefault(); result = Object.fromEntries(new FormData(form)); dialog.close(); };
    dialog.addEventListener('close', () => { form.reset(); dialog.remove(); resolve(result); }, {once:true});
    dialog.showModal();
  });
}
