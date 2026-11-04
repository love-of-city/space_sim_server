export function collectionLayoutError(catalog, taskId, randomized) {
  if (!catalog) return '';
  if (randomized) return catalog.layout_randomization?.available ? '' : '随机布局当前不可用，请联系管理员检查布局池。';
  const fixed = catalog.fixed_layout;
  if (fixed?.configured && !fixed.available) return '固定操作台未通过校验，请联系管理员检查配置。';
  const task = catalog.tasks?.find(item => item.id === taskId);
  if (task?.requires_color_layout && !fixed?.available) {
    return '该任务需要可区分孔位的固定操作台。请联系管理员配置，或选择已可用的随机布局。';
  }
  return '';
}
