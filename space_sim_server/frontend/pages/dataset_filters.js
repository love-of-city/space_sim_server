export function recordTime(record) {
  if (record.created) return Number(record.created) / 1e6;
  const parts = record.id?.match(/episode-(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2})(\d{2})/);
  return parts ? new Date(+parts[1], +parts[2] - 1, +parts[3], +parts[4], +parts[5], +parts[6]).getTime() : NaN;
}

export function filterRecords(records, {query = '', source = 'all', outcome = 'all', task = 'all', from = '', to = '', ids = null} = {}) {
  const term = query.trim().toLowerCase();
  const start = from ? new Date(`${from}T00:00:00`).getTime() : -Infinity;
  const end = to ? new Date(`${to}T00:00:00`) : null;
  if (end) end.setDate(end.getDate() + 1);
  return records.filter(record => {
    const time = recordTime(record);
    return `${record.id} ${record.task || ''} ${record.instruction || ''} ${record.owner || ''}`.toLowerCase().includes(term)
      && (source === 'all' || record.source === source)
      && (outcome === 'all' || record.outcome === outcome)
      && (task === 'all' || record.task === task)
      && (!from || time >= start) && (!end || time < end.getTime())
      && (!ids || ids.has(record.id));
  });
}

export function chartCategory(id) {
  if (/force|torque/.test(id)) return 'wrench';
  if (/joint|gripper/.test(id) || id === 'velocity') return 'joints';
  if (/position|orientation|attitude/.test(id)) return 'tool';
  return 'task';
}
