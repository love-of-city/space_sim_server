// Presentation only. Absolute offsets make packet loss harmless; no velocity
// remains active after release, blur or a disconnected video stream.
export class PreviewOrbit {
  constructor({send,available,stopRobot,storage}) {
    Object.assign(this,{send,available,stopRobot,storage});
    this.views=new Map();this.key='';this.yaw=0;this.pitch=0;this.dragging=false;
    try {
      const saved=JSON.parse(storage?.getItem('space-arm-preview-angles-v1')||'[]');
      for(const entry of saved) {
        if(Array.isArray(entry)&&typeof entry[0]==='string'&&Array.isArray(entry[1])&&
          entry[1].length===2&&entry[1].every(Number.isFinite))this.views.set(entry[0],entry[1]);
      }
    } catch (_) { /* Storage may be disabled or contain old data. */ }
  }
  select(key) {
    this.end();this.key=key;
    [this.yaw,this.pitch]=this.views.get(key)||[0,0];
  }
  save(){
    this.views.set(this.key,[this.yaw,this.pitch]);
    try {this.storage?.setItem('space-arm-preview-angles-v1',JSON.stringify([...this.views]));} catch (_) {}
  }
  restore(){if(this.available())this.send({BskPreviewOrbit:JSON.stringify({yaw:this.yaw,pitch:this.pitch})});}
  begin(button,x,y) {
    if(button!==1 || !this.available())return false;
    this.stopRobot();this.dragging=true;this.x=x;this.y=y;return true;
  }
  move(x,y) {
    if(!this.dragging)return;
    if(!this.available()){this.end();return;}
    const dx=x-this.x,dy=y-this.y;this.x=x;this.y=y;
    if(!Number.isFinite(dx+dy)||Math.hypot(dx,dy)>256)return;
    this.yaw=(this.yaw+dx*.25)%360;
    this.pitch=Math.max(-180,Math.min(180,this.pitch+dy*.25));
    this.save();
    this.send({BskPreviewOrbit:JSON.stringify({yaw:this.yaw,pitch:this.pitch})});
  }
  end(){this.dragging=false;}
  reset(){this.end();this.yaw=this.pitch=0;
    this.save();
    if(this.available())this.send({BskPreviewOrbit:JSON.stringify({yaw:0,pitch:0})});}
}
