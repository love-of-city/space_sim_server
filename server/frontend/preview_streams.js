/** Keep logical camera choices stable across the flight and arm manifests. */
export function resolvePreviewStreams(configured, availableIds, selectedId) {
  const available = Array.isArray(availableIds) ? new Set(availableIds) : null;
  const streams = configured.map(item => {
    const candidates = [item.id, ...(item.aliases || [])];
    const previous = candidates.includes(selectedId) ? selectedId : item.id;
    const id = available ? candidates.find(candidate => available.has(candidate)) || previous : previous;
    return {...item, id, selected: candidates.includes(selectedId), disabled: available !== null && !available.has(id)};
  });
  return {streams, selectedId: (streams.find(item => item.selected) || streams[0])?.id};
}
