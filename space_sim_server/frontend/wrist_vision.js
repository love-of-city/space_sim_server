// Display only: this module has no robot command/keyboard/control connection.
export function videoContentRect(rect,width,height) {
  const scale=Math.min(rect.width/width,rect.height/height);
  return {x:rect.x+(rect.width-width*scale)/2,y:rect.y+(rect.height-height*scale)/2,width:width*scale,height:height*scale};
}
const labels={lost:'未发现靶标',insufficient_markers:'可用靶标不足',ambiguous:'平面位姿存在歧义',
  reprojection_rejected:'重投影检查未通过',capture_error:'图像采集失败',error:'图像解算失败',waiting:'等待末端图像',stopped:'场景已停止'};
export function wristViewState(data,receivedAt,now,{orbitShifted=false,session='',visualProbe=false}={}) {
  const mode=visualProbe?'视觉闭环短程试验 · 丢失/过期即停':'仅观察，不控制机械臂';
  const invalid=text=>({valid:false,result:null,text:`视觉无效 · ${text}\n位置／姿态误差：—\n${mode}`});
  if(orbitShifted)return invalid('观察角度已偏移，请按 Home 复位');
  if(!data)return invalid('等待末端图像');
  if(session && data.session_id!==session)return invalid('等待新场景图像');
  const age=(data.age_s??Infinity)+(now-receivedAt)/1000;
  if(data.status==='stale' || (data.valid && age>Math.min(1.5,data.stale_after_s??1.5)))return invalid('结果已过期');
  if(!data.valid || !data.result?.alignment?.valid)return invalid(`识别失败：${labels[data.status]||'视觉处理暂不可用'}${data.status==='waiting' && data.reason?' · '+data.reason:''}`);
  const a=data.result.alignment,p=a.interface_position_display_m.map(x=>x*1000),r=a.rotation_error_display_deg;
  const signed=x=>`${x>=0?'+':''}${x.toFixed(2)}`;
  return {valid:true,result:data.result,text:
    `视觉有效 · ${data.marker_count}/4 靶标 · 帧 ${data.source_frame_id}\n`+
    `横向 X（右+）：${signed(p[0])} mm\n纵向 Y（下+）：${signed(p[1])} mm\n`+
    `前方间距：${p[2].toFixed(2)} mm\n相对 500 mm 准备位：${signed(a.position_error_display_mm[2])} mm\n`+
    `旋转向量 X / Y / Z（度）\n${r.map(signed).join(' / ')}\n`+
    `相机安装偏置已扣除 · ${Math.round(age*1000)} ms\n${mode}`};
}

export class WristVisionOverlay {
  constructor({viewport,canvas,panel,video,getContext,request,now=()=>performance.now()}) {
    Object.assign(this,{viewport,canvas,panel,video,getContext,request,now});
    this.data=null;this.receivedAt=0;this.nextPoll=0;this.busy=false;this.generation=0;this.active=false;
    this.timer=setInterval(()=>this.tick(),100);
  }
  clear(){this.canvas.getContext('2d').clearRect(0,0,this.canvas.width,this.canvas.height);}
  async poll(generation) {
    this.busy=true;
    try {
      const response=await this.request('/api/vision/wrist/status',{cache:'no-store',signal:AbortSignal.timeout(1000)});
      if(!response.ok)throw Error('status unavailable');
      const data=await response.json();
      if(generation===this.generation){this.data=data;this.receivedAt=this.now();}
    } catch(_) {
      if(generation===this.generation){this.data={valid:false,status:'error',reason:'视觉连接中断'};this.receivedAt=this.now();}
    } finally {this.busy=false;}
  }
  tick() {
    const context=this.getContext();
    const active=context.selected && context.visible;
    this.panel.hidden=this.canvas.hidden=!active;
    if(!active){if(this.active){this.generation++;this.data=null;this.clear();}this.active=false;return;}
    if(!this.active){this.generation++;this.data=null;this.nextPoll=0;}this.active=true;
    const now=this.now();
    if(!context.ready){this.generation++;this.data=null;this.panel.textContent='视觉无效 · 场景或视频未就绪\n位置／姿态误差：—';this.panel.dataset.valid='false';this.clear();return;}
    if(now>=this.nextPoll && !this.busy){this.nextPoll=now+200;void this.poll(this.generation);}
    const view=wristViewState(this.data,this.receivedAt,now,context);
    this.panel.textContent=view.text;this.panel.dataset.valid=String(view.valid);this.clear();
    if(view.valid)this.draw(view.result);
  }
  draw(result) {
    const video=this.video();
    if(!video || !video.videoWidth)return;
    if(video.videoWidth!==result.image_size[0] || video.videoHeight!==result.image_size[1]){
      this.panel.textContent='视觉无效 · 视频分辨率与标定不一致\n位置／姿态误差：—';this.panel.dataset.valid='false';return;
    }
    const parent=this.viewport.getBoundingClientRect(),rect=videoContentRect(video.getBoundingClientRect(),video.videoWidth,video.videoHeight);
    const dpr=window.devicePixelRatio||1;
    this.canvas.width=Math.round(parent.width*dpr);this.canvas.height=Math.round(parent.height*dpr);
    const ctx=this.canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.lineWidth=2;ctx.font='14px sans-serif';
    const point=p=>[rect.x-parent.x+p[0]/video.videoWidth*rect.width,rect.y-parent.y+p[1]/video.videoHeight*rect.height];
    for(const d of result.detections||[]){const ps=d.corners.map(point);ctx.strokeStyle='#36ff68';ctx.fillStyle='#36ff68';ctx.beginPath();ps.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.stroke();ctx.fillText(`ID ${d.id}`,ps[0][0],ps[0][1]-6);}
    const pixels=result.overlay_pixels;
    if(pixels){const m=point(pixels.interface_center),a=point(pixels.grip_axis_at_estimated_depth);ctx.strokeStyle='#22f2ff';ctx.beginPath();ctx.moveTo(a[0]-10,a[1]);ctx.lineTo(a[0]+10,a[1]);ctx.moveTo(a[0],a[1]-10);ctx.lineTo(a[0],a[1]+10);ctx.stroke();ctx.strokeStyle='#ffa629';ctx.beginPath();ctx.arc(m[0],m[1],5,0,2*Math.PI);ctx.stroke();}
  }
}
