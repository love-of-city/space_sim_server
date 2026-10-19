// Inputs are described by the backend's validated schema, including units/ranges.
export function serializeRandomization(entries) {
  return Object.fromEntries(entries.map(({key, enabled, fields}) => [key, {
    enabled: Boolean(enabled),
    ...Object.fromEntries(fields.map(({name, value, type}) => [name, type === 'string' ? value : Number(value)])),
  }]));
}

export function createRandomizationControls(host, catalog) {
  const entries=[];
  host.replaceChildren();
  const text=(tag,value,cls)=>{const el=document.createElement(tag);el.textContent=value;if(cls)el.className=cls;return el;};
  for(const option of catalog){
    const card=document.createElement('fieldset');card.className='randomization-card';
    const legend=document.createElement('legend'),label=document.createElement('label');label.className='randomization-toggle';
    const enabled=document.createElement('input');enabled.type='checkbox';enabled.defaultChecked=option.default.enabled;
    enabled.dataset.option=option.key;label.append(enabled,text('span',option.label));legend.append(label);card.append(legend);
    card.append(text('p',option.description,'muted'));
    const grid=document.createElement('div');grid.className='form-grid';
    const fields=option.fields.map(field=>{
      const wrapper=document.createElement('label');wrapper.className='field';wrapper.append(text('span',field.label));
      const input=document.createElement(field.enum?'select':'input');
      if(field.enum){for(const value of field.enum){const item=text('option',({ee_base:'机械臂腕部',cubesat_bus:'卫星本体',capture_target:'自由漂浮目标'})[value]||value);item.value=value;item.defaultSelected=value===field.default;input.append(item);}}
      else{input.type='number';input.min=field.minimum;input.max=field.maximum;input.step=field.step??'any';input.defaultValue=String(field.default);input.required=true;}
      input.dataset.parameter=field.name;wrapper.append(input);grid.append(wrapper);
      return {...field,input};
    });
    const sync=()=>{for(const field of fields)field.input.disabled=!enabled.checked;card.classList.toggle('is-enabled',enabled.checked);};
    enabled.addEventListener('change',sync);sync();card.append(grid);host.append(card);entries.push({key:option.key,enabled,fields,sync});
  }
  return {
    value(){return serializeRandomization(entries.map(entry=>({key:entry.key,enabled:entry.enabled.checked,fields:entry.fields.map(field=>({...field,value:field.input.value}))})));},
    sync(){entries.forEach(entry=>entry.sync());},
  };
}
