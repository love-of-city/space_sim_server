"""Read-only native wrist-image consumer with strict invalidation and freshness."""
import base64
import hashlib
import json
from pathlib import Path
import queue
import threading
import time
import os
from .vision_service import _Worker
from .wrist_vision_config import VISION_CAMERA_ID,build_wrist_config


class _WristWorker(_Worker):
    binary_input = False
    module='space_arm_platform.wrist_vision_worker'
    log_name='wrist_vision_worker.log'
    def __call__(self,png):
        return self._exchange({'png':base64.b64encode(png).decode('ascii')})


class WristVisionService:
    def __init__(self,root:Path,*,config=None,estimator=None,now=time.monotonic,wall_now_ns=time.time_ns,stale_s=1.5):
        self.root=Path(root);self.config=config;self.static_config=config is not None;self.estimator=estimator;self.now=now;self.stale_s=stale_s
        self.lock=threading.RLock();self.queue=queue.Queue(maxsize=1);self.stop=threading.Event()
        self.thread=None;self.epoch=0;self.session='';self.frame=-1;self.result=None
        self.status_name='waiting';self.reason='等待末端相机图像';self.received=None;self.processed=0;self.dropped=0
        self.latest_image=None;self.result_received=None
        self.wall_now_ns=wall_now_ns;self.saved_sessions=set()
        runtime=self.root/'run/vision_runtime.json'
        settings=json.loads(runtime.read_text('utf-8-sig')) if runtime.exists() else {}
        self.evaluation_dir=Path(settings['wrist_evaluation_directory']) if settings.get('wrist_evaluation_directory') else None
        self.export_path=self.root/'run/wrist_alignment_observation.json' if self.evaluation_dir else None
        self.last_evaluation_key=None
        self.export_lock=threading.Lock()
        self.evidence_dir=Path(settings['evidence_directory'])/'wrist' if settings.get('evidence_directory') else None

    def start(self):
        if self.thread and self.thread.is_alive():return
        self.stop.clear();self.thread=threading.Thread(target=self._work,daemon=True,name='wrist-vision');self.thread.start()

    def close(self):
        self.stop.set();self.invalidate('stopped')
        if self.thread:self.thread.join(timeout=1)
        if isinstance(self.estimator,_WristWorker):self.estimator.close()

    def _clear_queue(self):
        while True:
            try:self.queue.get_nowait();self.queue.task_done();self.dropped+=1
            except queue.Empty:break

    def invalidate(self,status='waiting',reason=''):
        with self.lock:
            self.epoch+=1;self.result=None;self.latest_image=None;self.result_received=None
            self.status_name=status;self.reason=reason or status;self._clear_queue()

    def _load(self):
        if self.config is None:
            self.config=build_wrist_config(self.root)
            path=self.root/'run/wrist_vision_config.json';path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(self.config),encoding='utf-8')
            self.estimator=_WristWorker(self.root,path)

    def on_capture(self,metadata,products):
        if metadata.get('camera_id')!=VISION_CAMERA_ID:return
        session=str(metadata.get('session_id',''));frame=int(metadata['source_frame_id'])
        if not session:raise ValueError('wrist capture requires session')
        with self.lock:
            if session!=self.session:
                self.invalidate();self.session=session;self.frame=-1
                if not self.static_config:
                    if isinstance(self.estimator,_WristWorker):self.estimator.close()
                    self.config=None;self.estimator=None
            self._load()
            if frame<=self.frame:return
            self.frame=frame;self.received=self.now()
            if self.evaluation_dir and frame%25==0 and products.get('rgb'):
                folder=self.evaluation_dir/hashlib.sha256(session.encode()).hexdigest()[:16]
                folder.mkdir(parents=True,exist_ok=True)
                (folder/f'{frame:08d}.png').write_bytes(products['rgb'])
                record={k:metadata.get(k) for k in ('session_id','source_frame_id','sim_time_ns','source_wall_time_ns','resolution','intrinsics')}
                record.update(received_wall_time_ns=str(self.wall_now_ns()),image=str(folder/f'{frame:08d}.png'))
                with (self.evaluation_dir/'capture_samples.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(record)+'\n')
            # The current local UE and BSK share the host wall clock. Include
            # transport/readback time, not merely time since API receipt.
            if metadata.get('source_wall_time_ns'):
                self.received-=max(0,(self.wall_now_ns()-int(metadata['source_wall_time_ns']))/1e9)
                if self.now()-self.received>self.stale_s:
                    self.invalidate('stale','图像源帧已过期');return
            if metadata.get('type')=='capture_error' or not products.get('rgb'):
                self.invalidate('capture_error',str(metadata.get('error','missing RGB')));return
            item=(self.epoch,session,frame,str(metadata['sim_time_ns']),self.received,products['rgb'])
            self._clear_queue()
            self.queue.put_nowait(item)

    def _work(self):
        while not self.stop.is_set():
            try:epoch,session,frame,sim,received,png=self.queue.get(timeout=.1)
            except queue.Empty:
                self.publish_evaluation();continue
            started=time.perf_counter()
            try:
                result=self.estimator(png)
                result.update(session_id=session,source_frame_id=str(frame),sim_time_ns=sim,
                              processing_ms=(time.perf_counter()-started)*1000)
                with self.lock:
                    if epoch==self.epoch and session==self.session:
                        self.result=result;self.status_name=result.get('status','error')
                        self.reason=result.get('reason','');self.result_received=received
                        self.latest_image=png;self.processed+=1
                        if self.evidence_dir and result.get('alignment',{}).get('valid') and session not in self.saved_sessions:
                            self.saved_sessions.add(session)
                            directory=self.evidence_dir/hashlib.sha256(session.encode()).hexdigest()[:16]
                            directory.mkdir(parents=True,exist_ok=True)
                            (directory/'native.png').write_bytes(png)
                            (directory/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
                            (directory/'calibration.json').write_text(json.dumps(self.config,indent=2),encoding='utf-8')
            except Exception as error:
                with self.lock:
                    if epoch==self.epoch:self.result=None;self.result_received=None;self.latest_image=None;self.status_name='error';self.reason=str(error)
            finally:
                self.publish_evaluation()
                self.queue.task_done()

    def publish_evaluation(self):
        """Opt-in local image-pose channel, with no simulator truth fields."""
        if self.export_path is None:return
        with self.export_lock:
            try:self._publish_evaluation_locked()
            except OSError:return  # Readers reject the previous packet by age.

    def _publish_evaluation_locked(self):
        state=self.status();state['published_wall_time_ns']=str(self.wall_now_ns())
        self.export_path.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.export_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(state,allow_nan=False),encoding='utf-8')
        os.replace(temporary,self.export_path)
        key=(state['session_id'],state['source_frame_id'],state['status'])
        if key!=self.last_evaluation_key:
            self.evaluation_dir.mkdir(parents=True,exist_ok=True)
            with (self.evaluation_dir/'perception_stream.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(state)+'\n')
            self.last_evaluation_key=key

    def wait_idle(self,timeout=3):
        end=time.monotonic()+timeout
        while self.queue.unfinished_tasks and time.monotonic()<end:time.sleep(.01)
        return not self.queue.unfinished_tasks

    def status(self):
        with self.lock:
            age=None if self.result_received is None else self.now()-self.result_received
            stale=age is not None and age>self.stale_s
            valid=not stale and self.result is not None and self.result.get('alignment',{}).get('valid') is True and self.status_name=='tracking'
            status='stale' if stale else self.status_name
            return dict(valid=valid,status=status,reason=self.reason,control_enabled=False,
                session_id=self.session,source_frame_id=self.result.get('source_frame_id') if self.result else str(self.frame),
                sim_time_ns=self.result.get('sim_time_ns') if self.result else None,
                age_s=age,stale_after_s=self.stale_s,processed_frames=self.processed,dropped_frames=self.dropped,
                result=self.result if valid else None,image_size=self.config.get('image_size') if self.config else [1024,1024],
                marker_count=len(self.result.get('detections',[])) if self.result and not stale else 0,
                calibration={k:self.config.get(k) for k in ('K','D','image_size','mount')} if self.config else None,
                algorithm_uses_simulator_truth=False)
