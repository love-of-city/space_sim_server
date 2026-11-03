// Resolve task labels and historical aliases from the backend's task catalog.
export function createTaskCatalog(api) {
  let catalog = null, pending = null;
  return {
    async load() {
      if (catalog) return catalog;
      if (!pending) {
        pending = Promise.resolve().then(() => api('/api/collections/catalog'))
          .then(data => { catalog = data; return data; })
          .finally(() => { pending = null; });
      }
      return pending;
    },
    label(identity) {
      if (!identity || identity === 'spacecraft arm teleoperation') return '遥操作采集';
      const name = identity.split(':')[0];
      const canonical = catalog?.task_aliases?.[name] || name;
      return catalog?.tasks?.find(task => task.id === canonical)?.label || identity;
    },
  };
}
