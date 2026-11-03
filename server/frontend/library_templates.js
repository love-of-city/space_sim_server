const labels = {
  '01_self_operation': '场景库 · 1 本星作业',
  '02_docked_operation': '场景库 · 2 双臂保持靶星作业',
  '03_rendezvous': '场景库 · 3 交会',
  '04_cross_transfer': '场景库 · 4 双臂跨星转运',
};

export function libraryFamilyId(taskId) {
  if (!taskId?.startsWith('library/')) return null;
  return 'library-family/' + taskId.split('/')[1];
}

export function groupSceneTemplates(templates) {
  const result = [], families = new Map();
  for (const item of templates) {
    const id = item.library_scene && libraryFamilyId(item.id);
    if (!id) { result.push(item); continue; }
    if (!families.has(id)) {
      const family = {...item, id, label: labels[item.id.split('/')[1]] || item.label,
        library_family: true, tasks: []};
      families.set(id, family); result.push(family);
    }
    families.get(id).tasks.push(item);
  }
  return result;
}

export function resolveSceneTask(templates, familyOrTemplateId, taskId) {
  const direct = templates.find(item => item.id === familyOrTemplateId);
  if (direct) return direct;
  const tasks = templates.filter(item => item.library_scene && libraryFamilyId(item.id) === familyOrTemplateId);
  return tasks.find(item => item.id === taskId) || tasks[0];
}
