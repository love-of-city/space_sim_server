"""Opt-in live Edge acceptance. Never launches the server or deletes datasets.

Cycles rotate through goals; first cancels/retries, second reloads/retries.
Each holds ready, exercises teleop, records, and inspects local artifacts.
FPS measures this browser, not a remote-network guarantee. No media decoding.
"""
from __future__ import annotations

import argparse
import base64
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time
from urllib.parse import urlsplit

import httpx
from websockets.sync.client import connect

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
from space_arm_platform.auth import AuthStore, SESSION_COOKIE

GOALS = [[0, -67.6, -86.6, 143.2, -85.5, 0], [12, -67.6, -86.6, 143.2, -85.5, 18],
         [-12, -67.6, -86.6, 143.2, -85.5, -18]]


def check(condition, message):
    if not condition:
        raise RuntimeError(message)


def wait(label, probe, seconds=180):
    deadline = time.monotonic() + seconds
    while True:
        result = probe()
        if result:
            return result
        check(time.monotonic() < deadline, 'Timeout: ' + label)
        time.sleep(0.2)


def finite(value):
    if isinstance(value, float):
        check(math.isfinite(value), 'Non-finite telemetry')
    elif isinstance(value, (dict, list)):
        for child in value.values() if isinstance(value, dict) else value:
            finite(child)


def vector(value):
    check(isinstance(value, list) and len(value) == 6 and all(
        type(item) in (int, float) and math.isfinite(item) for item in value), 'Invalid six-axis vector')
    return value


def delta(first, second):
    return max(abs(actual - previous) for previous, actual in zip(first, second))


PROBE = r"""(() => {
  const trace = window.__acceptance = {first:{},latest:null,start:null,reset:null,open:false,
    documentId:Math.random().toString(36)};
  const original = window.fetch;
  window.fetch = async (...args) => {
    const response = await original(...args);
    const path = new URL(typeof args[0]==='string'?args[0]:args[0].url,location.href).pathname;
    if (['/api/scenes/start','/api/scenes/reset'].includes(path)) {
      const data = await response.clone().json();
      trace[path.endsWith('start')?'start':'reset'] = {http:response.status,
        id:data.instance_id,generation:data.request_id,status:data.status};
    }
    return response;
  };
  const NativeSocket = window.WebSocket;
  window.WebSocket = class extends NativeSocket {
    constructor(...args) {
      super(...args);
      if(new URL(args[0],location.href).pathname!=='/ws/operator') return;
      this.addEventListener('open',()=>trace.open=true);
      this.addEventListener('close',()=>trace.open=false);
      this.addEventListener('message',event=>{
        const message=JSON.parse(event.data);
        if(message.type!=='observation') return;
        const observation=message.payload;
        const key=JSON.stringify([observation.scene_instance_id,observation.reset_generation]);
        if(!Object.hasOwn(trace.first,key)) trace.first[key]=observation;
        trace.latest=observation;
      });
    }
  };
})()"""


class CDP:
    def __init__(self, executable):
        self.executable, self.process, self.socket, self.profile = executable, None, None, None
        self.sequence, self.sockets, self.closed = 0, set(), set()
        self.connections = ExitStack()

    def start(self, origin, token):
        self.profile = tempfile.TemporaryDirectory(prefix='arm-acceptance-')
        self.process = subprocess.Popen([self.executable, '--headless=new', '--no-first-run',
            '--autoplay-policy=no-user-gesture-required', '--remote-debugging-port=0',
            '--remote-debugging-address=127.0.0.1', '--window-size=1440,1000',
            '--user-data-dir=' + self.profile.name, 'about:blank'], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        port_file = Path(self.profile.name) / 'DevToolsActivePort'
        wait('Edge debugging port', lambda: port_file.is_file(), 30)
        port = port_file.read_text().splitlines()[0]
        with httpx.Client(trust_env=False, timeout=3) as local:
            page = wait('CDP page', lambda: next((page for page in local.get(
                f'http://127.0.0.1:{port}/json').json() if page['type'] == 'page'), None), 30)
        self.socket = self.connections.enter_context(connect(
            page['webSocketDebuggerUrl'], proxy=None, ping_interval=None, max_size=16*1024*1024))
        self.command('Network.enable')
        self.command('Page.enable')
        self.command('Page.addScriptToEvaluateOnNewDocument', {'source': PROBE})
        check(self.command('Network.setCookie', {'name': SESSION_COOKIE, 'value': token,
            'url': origin, 'secure': origin.startswith('https:'), 'httpOnly': True}).get('success'), 'Cookie rejected')
        self.command('Page.navigate', {'url': origin + '/'})
        self.page_ready()

    def command(self, method, params=None):
        self.sequence += 1
        self.socket.send(json.dumps({'id': self.sequence, 'method': method, 'params': params or {}}))
        deadline = time.monotonic() + 4
        while True:
            check(time.monotonic() < deadline, 'CDP timeout: ' + method)
            message = json.loads(self.socket.recv(timeout=max(0.001, deadline - time.monotonic())))
            event = message.get('params', {})
            if message.get('method') == 'Network.webSocketCreated' and urlsplit(event['url']).path == '/ws/operator':
                self.sockets.add(event['requestId'])
            if message.get('method') == 'Network.webSocketClosed':
                self.closed.add(event['requestId'])
            if message.get('id') == self.sequence:
                check('error' not in message, 'CDP error: ' + method)
                return message.get('result', {})

    def evaluate(self, expression):
        result = self.command('Runtime.evaluate', {'expression': expression, 'returnByValue': True, 'awaitPromise': True})
        check('exceptionDetails' not in result, 'Browser JavaScript evaluation failed')
        return result.get('result', {}).get('value')

    def page_ready(self):
        wait('authenticated page', lambda: self.evaluate("Boolean(window.__acceptance?.open && "
            "document.getElementById('randomizationProfile')?.options.length>1)"), 45)

    def click(self, name):
        wait(name + ' enabled', lambda: self.evaluate(
            f"Boolean(document.getElementById('{name}') && !document.getElementById('{name}').disabled)"), 20)
        self.evaluate(f"document.getElementById('{name}').click()")

    def screenshot(self, path):
        path.write_bytes(base64.b64decode(self.command('Page.captureScreenshot', {'format': 'png'})['data']))

    def close(self):
        try:
            if self.socket:
                try:
                    self.command('Browser.close')
                except Exception:
                    pass
                self.connections.close()
        finally:
            try:
                if self.process and self.process.poll() is None:
                    try:
                        self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=5)
            finally:
                if self.profile:
                    self.profile.cleanup()


class Tester:
    def __init__(self, args, client, browser, report):
        self.args, self.client, self.browser, self.report = args, client, browser, report
        self.scene, self.episode, self.cycle, self.generation = None, None, None, None
        self.pending_recording, self.pending_start = False, False

    def api(self, method, path, payload=None):
        response = self.client.request(method, path, json=payload, timeout=4 if path == '/api/state' else 90)
        check(response.is_success, f'{method} {path}: HTTP {response.status_code}')
        return response.json()

    def state(self):
        return self.api('GET', '/api/state')

    def stop(self, replace=False):
        state = self.state()
        check(not state.get('active_episode'), 'Refusing to stop scene with active recording')
        runtime = state['scene_runtime']
        if runtime.get('active'):
            check((runtime.get('instance') or {}).get('instance_id') == self.scene or replace,
                  'Refusing to stop an unowned scene')
            self.api('POST', '/api/scenes/stop')
            wait('scene stopped', lambda: not self.state()['scene_runtime'].get('active'), 30)
        self.scene = None

    def sample(self, changing_generation=False):
        state = self.state()
        runtime, simulation = state['scene_runtime'], state['simulation']
        check(runtime.get('phase') != 'failed', 'Scene failed')
        check((runtime.get('instance') or {}).get('instance_id') == self.scene, 'Test scene replaced')
        observation = simulation.get('latest_observation') or {}
        if observation.get('scene_instance_id') != self.scene:
            return None
        finite(observation)
        check(simulation.get('connected'), 'Simulation disconnected')
        position = vector(observation.get('arm_joint_position_rad'))
        velocity = vector(observation.get('arm_joint_velocity_rad_s') or observation.get('joint_velocity_rad_s', [])[:6])
        preparation = observation.get('arm_preparation') or {}
        check(preparation.get('status') != 'failed', 'Preparation failed')
        if self.generation is not None and not changing_generation:
            check(observation.get('reset_generation') == self.generation, 'Unexpected reset generation')
        sample = {'monotonic': time.monotonic(), 'sim_time_ns': int(observation['sim_time_ns']),
                  'generation': observation.get('reset_generation'), 'position': position,
                  'velocity': velocity, 'preparation': preparation}
        self.cycle['samples'].append(sample)
        return sample

    def zero(self, sample):
        key = json.dumps([self.scene, sample['generation']], separators=(',', ':'))
        first = self.browser.evaluate('window.__acceptance.first[' + json.dumps(key) + '] || null')
        if first is None:
            return False
        finite(first)
        check(max(abs(value) for value in vector(first.get('arm_joint_position_rad'))) < 1e-6,
              'First matching browser telemetry was not measured zero')
        check(first.get('arm_preparation', {}).get('status') not in ('moving', 'settling', 'ready'),
              'First zero was observed after motion began')
        self.cycle['zero_observations'].append(first)
        return True

    def ready(self, sample, target=True):
        preparation = sample['preparation']
        check(preparation.get('status') != 'cancelled', 'Unexpected cancellation')
        if preparation.get('status') != 'ready' or preparation.get('ready') is not True:
            return False
        if target:
            check(delta(vector(preparation.get('goal_deg')), self.cycle['goal']) < 1e-6, 'Wrong goal')
            check(delta([math.degrees(value) for value in sample['position']], self.cycle['goal']) <= 0.8
                  and max(abs(value) for value in sample['velocity']) <= 0.02, 'Ready measured bounds exceeded')
            check(isinstance(preparation.get('max_error_deg'), (float, int))
                  and 0 <= preparation['max_error_deg'] <= 0.8
                  and isinstance(preparation.get('max_velocity_rad_s'), (float, int))
                  and 0 <= preparation['max_velocity_rad_s'] <= 0.02, 'Ready reported bounds exceeded')
        return True

    def gate(self):
        check(not self.state().get('active_episode'), 'Existing recording must not be interrupted')
        sample = self.sample()
        check(sample and not sample['preparation'].get('ready'), 'Gate must be tested before ready')
        self.pending_recording = True
        response = self.client.post('/api/episodes/start', json=self.payload())
        self.cycle['gate_http_status'] = response.status_code
        if response.is_success:
            self.episode = response.json()['episode_id']
        check(response.status_code == 409, 'Dataset gate did not return HTTP 409')
        self.pending_recording = False

    def payload(self):
        return {'task': 'arm preparation infrastructure acceptance', 'tags': ['infrastructure', 'acceptance']}

    def hold(self, seconds, target=True, recording=False):
        started, previous, progress = time.monotonic(), None, {}
        next_progress = started + 60
        while True:
            sample = self.sample()
            check(sample and self.ready(sample, target), 'Lost ready during hold')
            dom = self.browser.evaluate("""({status:document.getElementById('armPreparationStatus').textContent,
              collect:!document.getElementById('startEpisode').disabled,open:window.__acceptance.open,
              latest:window.__acceptance.latest,videos:Array.from(document.querySelectorAll('video')).map(
              (video,index)=>({id:video.id||String(index),width:video.videoWidth,height:video.videoHeight,
              frames:video.getVideoPlaybackQuality().totalVideoFrames,time:video.currentTime}))})""")
            finite(dom)
            latest = dom.pop('latest') or {}
            check(latest.get('scene_instance_id') == self.scene and latest.get('reset_generation') == self.generation
                  and latest.get('arm_preparation', {}).get('ready') is True, 'Browser telemetry mismatch')
            check(dom['open'] and dom['status'] and dom['collect'] == (not recording), 'DOM recording gate mismatch')
            check(dom['videos'] and all(video['width'] and video['height'] for video in dom['videos']), 'No decoded video')
            now = time.monotonic()
            counters = {'simulation': sample['sim_time_ns'], 'browser_simulation': int(latest['sim_time_ns'])}
            counters.update({'video:' + video['id']: video['frames'] for video in dom['videos']})
            row = {'monotonic': now, 'dom': dom, 'counters': counters}
            if previous:
                interval = now - previous['monotonic']
                check(interval <= 5 and counters.keys() == previous['counters'].keys(), 'Sampling gap or video replacement')
                row['interval_seconds'] = interval
                row['deltas'] = {key: value - previous['counters'][key] for key, value in counters.items()}
                row['fps'] = {key: change / interval for key, change in row['deltas'].items() if key.startswith('video:')}
                for key, change in row['deltas'].items():
                    check(change >= 0 and now - progress[key] <= 5, 'Counter regressed or stalled over 5s: ' + key)
                    if change > 0:
                        progress[key] = now
            else:
                progress = {key: now for key in counters}
                initial_counters = counters.copy()
            self.cycle['hold_samples'].append(row)
            if now >= next_progress:
                print(f"Cycle {self.cycle['cycle']}: holding {now - started:.0f}/{seconds:.0f}s", flush=True)
                next_progress = now + 60
            if now - started >= seconds:
                check(previous is not None, 'No hold intervals')
                check(all(value > initial_counters[key] for key, value in counters.items()), 'Hold did not advance simulation/video')
                return
            previous = row
            time.sleep(min(1, max(0, seconds - (time.monotonic() - started))))

    def interrupt(self, moving, reload_page):
        started = time.monotonic()
        old_document = self.browser.evaluate('window.__acceptance.documentId')
        old_sockets = self.browser.sockets - self.browser.closed
        if reload_page:
            check(old_sockets, 'No operator WebSocket before reload')
            self.browser.command('Page.reload', {'ignoreCache': True})
        else:
            self.browser.click('cancelPrepareArm')

        def stopped():
            sample = self.sample()
            if sample:
                check(math.degrees(delta(moving['position'], sample['position'])) <= 5, 'Stopping travel exceeded 5 degrees')
                if sample['preparation'].get('status') == 'cancelled' and max(abs(value) for value in sample['velocity']) <= 0.02:
                    return sample
            return False

        stopped_sample = wait('cancelled and stopped', stopped, 5)
        check(time.monotonic() - started <= 5, 'Stopping took over 5 seconds')
        self.cycle['stop_seconds'] = time.monotonic() - started
        print(f"Cycle {self.cycle['cycle']}: interruption stopped in {self.cycle['stop_seconds']:.2f}s", flush=True)
        if reload_page:
            self.browser.page_ready()
            check(self.browser.evaluate('window.__acceptance.documentId') != old_document, 'Reload did not replace document')
            wait('actual operator disconnect/reconnect', lambda: self.browser.evaluate('true')
                 and old_sockets <= self.browser.closed
                 and bool(self.browser.sockets - old_sockets - self.browser.closed), 10)
        self.no_replay(stopped_sample, 'cancelled')
        self.gate()
        self.browser.evaluate('window.__acceptance.reset=null')
        self.browser.click('resetScene')
        reset = wait('reset response', lambda: self.browser.evaluate('window.__acceptance.reset'), 65)
        check(reset['http'] == 200 and reset.get('status') == 'completed' and reset.get('generation')
              and reset['generation'] != self.generation, 'Reset failed or reused generation')
        self.generation = reset['generation']
        fresh = wait('fresh reset telemetry', lambda: self.reset_sample(), 20)
        wait('first reset zero', lambda: self.zero(fresh), 10)
        check(max(abs(value) for value in fresh['position']) < 1e-6, 'Reset is not measured zero')
        self.no_replay(fresh, 'waiting')
        self.browser.click('prepareArm')
        self.cycle['explicit_retry'] = True

    def reset_sample(self):
        sample = self.sample(changing_generation=True)
        return sample if sample and sample['generation'] == self.generation else None

    def no_replay(self, baseline, status):
        deadline = time.monotonic() + 6
        while True:
            sample = self.sample()
            check(sample and sample['preparation'].get('status') == status
                  and math.degrees(delta(baseline['position'], sample['position'])) <= 0.1
                  and max(abs(value) for value in sample['velocity']) <= 0.02, 'Unexpected replay, rearm, or drift')
            if time.monotonic() >= deadline:
                return
            time.sleep(0.2)

    def teleop(self):
        before = self.sample()
        try:
            point = self.browser.evaluate("""(()=>{const element=document.getElementById('viewport');
              element.scrollIntoView();const rect=element.getBoundingClientRect();
              return {x:rect.x+rect.width/2,y:rect.y+rect.height/2};})()""")
            for kind in ('mousePressed', 'mouseReleased'):
                self.browser.command('Input.dispatchMouseEvent', dict(point, type=kind, button='left', clickCount=1))
            wait('operation mode', lambda: self.browser.evaluate(
                "document.getElementById('viewport').getAttribute('aria-pressed')==='true'"), 10)
            self.browser.command('Input.dispatchKeyEvent', {'type': 'keyDown', 'code': 'KeyD', 'key': 'd', 'windowsVirtualKeyCode': 68})
            time.sleep(0.2)
        finally:
            try:
                self.browser.command('Input.dispatchKeyEvent', {'type': 'keyUp', 'code': 'KeyD', 'key': 'd', 'windowsVirtualKeyCode': 68})
            finally:
                for kind in ('keyDown', 'keyUp'):
                    self.browser.command('Input.dispatchKeyEvent', {'type': kind, 'code': 'Escape', 'key': 'Escape', 'windowsVirtualKeyCode': 27})
        def changed():
            after = self.sample()
            return after if after and after['sim_time_ns'] > before['sim_time_ns'] and delta(before['position'], after['position']) > 1e-6 else None
        after = wait('measured teleop motion', changed, 10)
        self.cycle['teleop'] = {'delta_rad': delta(before['position'], after['position']),
                               'sim_time_delta_ns': after['sim_time_ns'] - before['sim_time_ns']}

    def record(self):
        check(not self.state().get('active_episode'), 'Existing recording must not be interrupted')
        self.pending_recording = True
        episode = self.api('POST', '/api/episodes/start', self.payload())
        self.episode = episode['episode_id']
        check(Path(self.episode).name == self.episode and self.episode.startswith('episode-'), 'Unsafe episode ID')
        directory = self.args.episodes_root / self.episode
        self.cycle['episode'] = {'episode_id': self.episode, 'expected_directory': str(directory),
                                'expected_dataset_directory': str(directory / 'lerobot'), 'validation': 'pending'}
        wait('browser recording gate', lambda: self.browser.evaluate(
            "document.getElementById('startEpisode').disabled"), 10)
        self.hold(self.args.record_seconds, target=False, recording=True)
        episode = self.api('POST', '/api/episodes/stop', {'outcome': 'unknown', 'note': 'Infrastructure acceptance, not a grasp demonstration'})
        self.episode, self.pending_recording = None, False
        metadata = json.loads((directory / 'metadata.json').read_text(encoding='utf-8'))
        info = json.loads((directory / 'lerobot/meta/info.json').read_text(encoding='utf-8'))
        platform = json.loads((directory / 'lerobot/meta/platform.json').read_text(encoding='utf-8'))
        check(episode.get('dataset_status') == metadata.get('dataset_status') == 'complete'
              and not metadata.get('dataset_error'), 'Recording dataset did not complete')
        frames = metadata.get('dataset_frame_count', 0)
        check(frames > 0 and frames == episode.get('dataset_frame_count') == info.get('total_frames'), 'Empty or inconsistent frame counts')
        check(info.get('codebase_version') == 'v3.0' and info.get('total_episodes') == 1
              and info.get('fps') == metadata.get('dataset_fps') and 'rgb' in metadata.get('dataset_capture_products', []), 'Invalid dataset summary')
        check(metadata['episode_id'] == self.cycle['episode']['episode_id']
              and metadata.get('scene_instance', {}).get('instance_id') == self.scene, 'Artifact identity mismatch')
        check(set(self.payload()['tags']) <= set(metadata.get('tags', [])), 'Infrastructure tags missing')
        cameras = platform.get('camera_keys', {})
        check(len(cameras) == 2 and set(cameras) == set(metadata.get('dataset_camera_ids', [])), 'Expected two RGB cameras')
        videos = []
        for key in cameras.values():
            check(isinstance(key, str) and key.replace('_', '').isalnum(), 'Unsafe camera key')
            files = list((directory / 'lerobot/videos' / ('observation.images.' + key)).rglob('*.mp4'))
            check(files and all(path.stat().st_size > 0 for path in files), 'Missing/empty camera RGB video')
            videos.extend(str(path.relative_to(directory)) for path in files)
        check(any((directory / 'lerobot/data').rglob('*.parquet')), 'No dataset Parquet')
        self.cycle['episode'].update(validation='passed', dataset_status='complete', frame_count=frames, videos=videos)
        wait('browser recording finished', lambda: self.browser.evaluate(
            "!document.getElementById('startEpisode').disabled"), 10)
        print(f"Cycle {self.cycle['cycle']}: dataset complete, {frames} frames", flush=True)

    def run(self):
        if self.state()['scene_runtime'].get('active'):
            check(self.args.allow_replace_scene, 'Existing scene requires --allow-replace-scene')
            self.stop(replace=True)
        for index in range(self.args.cycles):
            goal = self.args.goals[index % len(self.args.goals)]
            self.generation = None
            self.cycle = {'cycle': index + 1, 'goal': goal, 'status': 'running', 'samples': [], 'hold_samples': [], 'zero_observations': []}
            self.report['cycles'].append(self.cycle)
            print(f"Cycle {index + 1}: starting goal {goal}", flush=True)
            wait('scene form unlocked', lambda: self.browser.evaluate("!document.getElementById('startScene').disabled"), 30)
            check(not self.state().get('active_episode') and not self.state()['scene_runtime'].get('active'), 'Scene/recording appeared before start')
            self.browser.evaluate("""(()=>{window.__acceptance.start=null;
              const profile=document.getElementById('randomizationProfile');profile.value='teleop-zero-prepare-v2';
              profile.dispatchEvent(new Event('change',{bubbles:true}));
              %s.forEach((value,index)=>document.getElementById('operatingJoint'+(index+1)).value=String(value));
              document.getElementById('sceneSeed').value='123';
              document.getElementById('sceneDatasetCapture').checked=true;})()""" % json.dumps(goal))
            self.pending_start = True
            self.browser.click('startScene')
            response = wait('browser Start response', lambda: self.browser.evaluate('window.__acceptance.start'))
            check(response['http'] == 200 and response.get('id'), 'Browser Start failed')
            self.scene, self.pending_start = response['id'], False
            self.cycle['scene_instance_id'] = self.scene
            instance = self.state()['scene_runtime']['instance']
            check(instance.get('arm_preparation_required') is True and instance.get('randomization_profile') == 'teleop-zero-prepare-v2'
                  and instance.get('operating_arm_joint_position_deg') == goal, 'Scene profile/goal mismatch')
            first = wait('matching scene telemetry', self.sample)
            self.generation = first['generation']
            wait('first observed zero', lambda: self.zero(first), 10)
            self.gate()
            def moving():
                sample = self.sample()
                check(not sample or sample['preparation'].get('status') not in ('cancelled', 'ready'), 'Missed moving state')
                return sample if sample and sample['preparation'].get('status') == 'moving' and max(abs(math.degrees(value)) for value in sample['position']) > 0.5 else None
            motion = wait('motion over 0.5 degrees', moving)
            if index < 2:
                self.interrupt(motion, reload_page=index == 1)
            wait('measured ready', lambda: (sample := self.sample()) and self.ready(sample))
            wait('browser ready/video', lambda: self.browser.evaluate("!document.getElementById('startEpisode').disabled && "
                 "document.querySelectorAll('video').length>0 && Array.from(document.querySelectorAll('video')).every(video=>video.videoWidth>0)"), 45)
            print(f"Cycle {index + 1}: measured ready, checking video and simulation continuity", flush=True)
            self.hold(self.args.hold_seconds if index == self.args.cycles - 1 else 6)
            self.browser.screenshot(self.args.output / f'cycle-{index + 1}-ready.png')
            self.teleop()
            self.record()
            self.hold(2, target=False)
            self.cycle['status'] = 'passed'
            if index < self.args.cycles - 1:
                self.stop()
        self.report['last_scene_running_ready'] = self.scene

    def cleanup(self):
        if self.cycle:
            self.cycle['status'] = 'failed'
        if self.pending_start and not self.scene:
            response = self.browser.evaluate('window.__acceptance?.start')
            self.scene = response.get('id') if response else None
        if self.scene:
            state = self.state()
            check((state['scene_runtime'].get('instance') or {}).get('instance_id') == self.scene, 'Cleanup refused replacement scene')
            if state.get('active_episode'):
                check(state['active_episode'] == self.episode, 'Cleanup refused unrelated recording')
                result = self.api('POST', '/api/episodes/stop', {'outcome': 'aborted', 'note': 'Infrastructure acceptance failed; artifacts retained'})
                self.report['cleanup_episode_id'] = result.get('episode_id')
            self.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--origin', required=True)
    parser.add_argument('--auth-database', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('run') / f'arm-preparation-{time.time_ns()}')
    parser.add_argument('--browser', help='Edge executable; otherwise locate installed Edge')
    parser.add_argument('--cycles', type=int, default=3, help='Total cycles; goals rotate in order')
    parser.add_argument('--hold-seconds', type=float, default=120, help='Final cycle ready hold')
    parser.add_argument('--record-seconds', type=float, default=5)
    parser.add_argument('--goals', default=json.dumps(GOALS), help='JSON array of six-angle degree arrays')
    parser.add_argument('--allow-replace-scene', action='store_true')
    parser.add_argument('--episodes-root', type=Path, default=ROOT / 'data/episodes', help='Read-only local artifact root')
    args = parser.parse_args()
    origin = urlsplit(args.origin)
    if origin.scheme not in ('http', 'https') or not origin.hostname or origin.username or origin.password or origin.path not in ('', '/') or origin.query or origin.fragment:
        parser.error('--origin must be an HTTP(S) origin without credentials/path/query/fragment')
    args.origin, args.output = args.origin.rstrip('/'), args.output.resolve()
    if not args.output.is_relative_to(ROOT / 'run') or args.output == ROOT / 'run':
        parser.error('--output must be a new directory under repository run/')
    try:
        args.goals = json.loads(args.goals)
        check(isinstance(args.goals, list) and args.goals, 'Empty goals')
        for goal in args.goals:
            vector(goal)
        check(args.cycles > 0 and all(math.isfinite(value) and value > 0 for value in (args.hold_seconds, args.record_seconds)), 'Durations/cycles must be positive')
    except (ValueError, RuntimeError) as error:
        parser.error(str(error))
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'cycles': [], 'cleanup_errors': [], 'origin': args.origin,
              'limitations': ['Local browser FPS only; no remote-network guarantee.', 'Metadata/file presence only; no full per-frame decoding.'],
              'skipped_assertions': ['Not all default scenarios/goals exercised'] if args.cycles < max(3, len(args.goals)) else []}
    auth = client = browser = tester = token = None
    success = False
    try:
        candidates = [args.browser] if args.browser else [shutil.which('msedge'), shutil.which('microsoft-edge')] + [
            str(Path(os.environ[variable]) / 'Microsoft/Edge/Application/msedge.exe')
            for variable in ('PROGRAMFILES(X86)', 'PROGRAMFILES', 'LOCALAPPDATA') if variable in os.environ]
        executable = next((path for path in candidates if path and Path(path).is_file()), None)
        check(executable, 'Edge not found; supply --browser')
        database = args.auth_database.resolve()
        check(database.is_file(), 'Auth database must already exist')
        with sqlite3.connect(database.as_uri() + '?mode=ro', uri=True) as existing:
            row = existing.execute("SELECT user_id FROM users WHERE role='admin' AND active=1 LIMIT 1").fetchone()
        check(row, 'Active administrator required; no bootstrap permitted')
        auth = AuthStore(database, 'unused-bootstrap', secrets.token_urlsafe(32))
        token, _ = auth.create_session(row[0])
        client = httpx.Client(base_url=args.origin, headers={'Origin': args.origin}, trust_env=False, timeout=90)
        client.cookies.set(SESSION_COOKIE, token)
        browser = CDP(executable)
        tester = Tester(args, client, browser, report)
        state = tester.state()
        check(not state.get('active_episode'), 'Existing recording must not be interrupted')
        check(not state['scene_runtime'].get('active') or args.allow_replace_scene, 'Existing scene requires --allow-replace-scene')
        browser.start(args.origin, token)
        tester.run()
        success = True
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        actions = []
        if not success and tester:
            actions.append(('scene/recording', tester.cleanup))
        if not success and browser and browser.socket:
            actions.append(('failure screenshot', lambda: browser.screenshot(args.output / 'failed.png')))
        actions.extend([('browser', browser.close if browser else None), ('HTTP', client.close if client else None),
                        ('session', (lambda: auth.delete_session(token)) if auth else None), ('auth', auth.close if auth else None)])
        for label, action in actions:
            if action:
                try:
                    action()
                except Exception as error:
                    report['cleanup_errors'].append(label + ': ' + str(error))
        report['status'] = 'passed' if success and not report['cleanup_errors'] else 'failed'
        text = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)
        (args.output / 'result.json').write_text(text.replace(token, '[REDACTED]') if token else text, encoding='utf-8')
    print(f"Acceptance {report['status']}: {args.output / 'result.json'}")
    for cycle in report['cycles']:
        if episode := cycle.get('episode'):
            print(f"Episode {episode['episode_id']}: {episode['expected_directory']}")
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
