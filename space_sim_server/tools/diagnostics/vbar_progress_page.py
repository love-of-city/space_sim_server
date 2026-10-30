"""Display an existing native trial without modifying simulation inputs.

Writes a local HTML page with automatic refresh and references to the original
captured JPEG pair. Reads only the final chunk of the potentially large trace.
"""
import argparse
from datetime import datetime, timedelta, timezone
from html import escape
import json
from pathlib import Path
import time


CHINA = timezone(timedelta(hours=8))
PHASES = {
    'VbarAcquire': 'GNSS入口归线与停稳', 'NearAcquire': 'GNSS入口归线与停稳',
    'TargetPointing': '17.4 m停泊调姿',
    'GNSSBridge5p5Approach': 'GNSS引导，接近5.5 m停泊点',
    'GNSSBridge5p5Hold': '5.5 m停泊检查',
    'GNSSBridgeNearApproach': '继续沿V-bar接近，尝试识别靶标',
    'OriginalStereoAcquire': '近距停稳，等待双目连续有效',
    'VisualApproach': '双目参与导航，V-bar继续控制接近',
    'PlatformEntry': '0.5 m作业点入口检查', 'PlatformHold': '0.5 m平台稳定驻留',
    'Grasp': '机械臂接近、接触和抓持力检查', 'Complete': '运动序列结束，等待独立验收',
    'TerminalHold': '作业点保持', 'SafeHold': '已停止继续接近',
}


def read_json(path):
    try:
        return json.loads(path.read_text('utf-8-sig'))
    except (OSError, ValueError):
        return {}


def last_row(path):
    try:
        with path.open('rb') as stream:
            stream.seek(max(0, path.stat().st_size-250_000))
            lines = stream.read().splitlines()
        for line in reversed(lines):
            try:
                row = json.loads(line)
                if isinstance(row, dict) and 'time_s' in row:
                    return row
            except ValueError:
                continue
    except OSError:
        pass
    return {}


def phase_label(phase):
    if phase in PHASES:
        return PHASES[phase]
    for prefix, name in [('Vbar50m', '50 m质心停泊点'), ('Optical17p4m', '17.4 m停泊点')]:
        if phase.startswith(prefix):
            return ('正在前往' if phase.endswith('Approach') else '正在减速到达'
                    if phase.endswith('Brake') else '正在检查')+name
    return '正在初始化仿真与图像链路'


def duration(seconds):
    minutes, seconds = divmod(max(0., float(seconds)), 60)
    return f'{int(minutes):02d}:{seconds:04.1f}'


def render(directory):
    row = last_row(directory/'trajectory.jsonl')
    live = read_json(directory/'live_diagnostics.json')
    acceptance = read_json(directory/'acceptance.json')
    failure = read_json(directory/'failure.json')
    policy = row.get('policy') or {}
    phase = policy.get('phase', live.get('phase', 'Initializing'))
    stamp = row.get('time_s', live.get('simulation_time_s', 0))
    depth = row.get('true_optical_depth_m', live.get('optical_depth_m'))
    reason = policy.get('failure_reason') or failure.get('reason') or ''
    status, tone = '正在运行', 'blue'
    if acceptance:
        status, tone = ('本轮验收通过', 'green') if acceptance.get('passed') else ('本轮验收未通过', 'red')
    elif reason or live.get('status') == 'failed_before_summary':
        status, tone = '本轮已停止，尚未通过验收', 'red'
    elif live.get('status') == 'finished':
        status, tone = '运行结束，正在核对验收结果', 'blue'
    gates = policy.get('vbar_nodes', {})
    capture = row.get('capture') or {}
    checks = [
        ('入口归线与停稳', (policy.get('acquire') or {}).get('completed', False)),
        ('50 m停泊 · 连续5 s', (gates.get('Vbar50m') or {}).get('completed', False)),
        ('17.4 m停泊 · 连续5 s', (gates.get('Optical17p4m') or {}).get('completed', False)),
        ('目标调姿并稳定保持', (policy.get('target_pointing') or {}).get('completed', False)),
        ('5.5 m停泊 · 连续5 s', (gates.get('GNSS5p5m') or {}).get('completed', False)),
        ('真实双目连续就绪', (gates.get('OriginalStereoAcquire') or {}).get('completed', False)),
        ('0.5 m作业点入口 · 连续5 s', (gates.get('PlatformStandoff') or {}).get('completed', False)),
        ('0.5 m平台驻留 · 连续5 s', (policy.get('platform_hold') or {}).get('completed', False)),
        ('四爪承载、抓持力合格 · 连续20 s', capture.get('completed', False)),
    ]
    steps = []
    active_seen = False
    for label, done in checks:
        active = not done and not active_seen
        if active:
            active_seen = True
        kind = 'done' if done else 'active' if active and tone != 'red' else 'pending'
        text = '已完成阶段检查' if done else '当前阶段' if kind == 'active' else '等待执行'
        steps.append(f'<li class="{kind}"><span class="dot"></span><span>{escape(label)}</span>'
                     f'<small>{text}</small></li>')
    image = row.get('image') or (read_json(directory/'image_progress.json').get('last') or {})
    frame = str(image.get('frame_id', ''))
    image_stamp = image.get('time_s', 0)
    pictures = ''
    if frame.isascii() and frame.isdecimal():
        for eye, label in [('left', '左目原图'), ('right', '右目原图')]:
            relative = f'actual_images/images/{frame}_{eye}.jpg'
            if (directory/relative).is_file():
                pictures += f'<figure><img src="{relative}" alt="{label}"><figcaption>{label}</figcaption></figure>'
    if not pictures:
        pictures = '<p class="muted">等待本轮第一组实采双目图像。</p>'
    visual = '本帧已被导航接受' if image.get('ekf_accepted') else '本帧没有有效视觉测量'
    freshness = f'图像采样时间 {duration(image_stamp)}，距当前仿真时刻 {max(0., stamp-image_stamp):.2f} s'
    updated = datetime.now(CHINA).strftime('%Y-%m-%d %H:%M:%S')
    try:
        age = max(0., time.time()-(directory/'trajectory.jsonl').stat().st_mtime)
    except OSError:
        age = 0
    waiting = '<p class="notice">暂未收到下一条仿真记录，正在等待更新。</p>' if age > 20 and not acceptance else ''
    warning = f'<p class="error">停止原因：{escape(str(reason))}</p>' if reason else ''
    final_checks = ''
    if acceptance:
        items = ''.join(f'<li>{escape(name)}：{"通过" if value.get("passed") else "未通过"}</li>'
                        for name, value in acceptance.get('checks', {}).items())
        final_checks = f'<section class="panel"><h2>独立物理验收结果</h2><ul>{items}</ul></section>'
    depth_text = '—' if depth is None else f'{depth:.3f} m'
    return f'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="2"><title>交会对接验收进度</title>
<style>
*{{box-sizing:border-box}}body{{margin:0;background:#0c1424;color:#e9effa;font:16px/1.6 "Microsoft YaHei",sans-serif}}
main{{max-width:1440px;margin:auto;padding:30px}}h1{{font-size:30px;margin:0 0 4px}}h2{{font-size:19px;margin:0 0 16px}}
.muted,small{{color:#a4b4cc}}.top{{display:flex;justify-content:space-between;align-items:center;gap:16px}}
.badge{{border-radius:24px;padding:8px 18px;font-weight:700;white-space:nowrap}}.blue{{background:#12375d;color:#93caff}}
.green{{background:#154536;color:#8bedd0}}.red{{background:#582832;color:#ffc1c9}}
.cards{{display:grid;grid-template-columns:2fr 1fr 1fr;gap:16px;margin:24px 0}}
.panel,.card{{background:#142139;border:1px solid #263953;border-radius:16px;padding:22px}}
.card strong{{display:block;font-size:25px;margin-top:8px}}.columns{{display:grid;grid-template-columns:1fr 1.6fr;gap:20px}}
.steps{{list-style:none;margin:0;padding:0}}.steps li{{display:flex;gap:12px;align-items:center;padding:13px 0;border-top:1px solid #273953}}
.steps small{{margin-left:auto;font-size:12px;white-space:nowrap}}.dot{{width:10px;height:10px;border-radius:50%;background:#53617b;flex-shrink:0}}
.done .dot{{background:#55dbb0}}.done{{color:#bceede}}.active .dot{{background:#65b8ff;box-shadow:0 0 0 5px #244b71}}.active{{color:#a6d7ff}}
.pictures{{display:grid;grid-template-columns:1fr 1fr;gap:12px}}figure{{margin:0}}img{{width:100%;aspect-ratio:1;object-fit:contain;background:#05080e;border-radius:10px}}
figcaption{{text-align:center;color:#bdcce1;margin-top:8px}}.error,.notice{{padding:12px 18px;background:#493226;border-radius:10px}}
.foot{{font-size:13px;margin-top:20px;color:#9caec7}}.panel+ .panel{{margin-top:18px}}
@media(max-width:1000px){{.columns{{grid-template-columns:1fr}}.cards{{grid-template-columns:1fr}}main{{padding:18px}}.top{{align-items:flex-start}}}}
</style></head><body><main>
<header class="top"><div><h1>交会对接验收进度</h1><div class="muted">本轮：从110 m初始化，验证V-bar、原双目和机械臂抓取。</div></div>
<span class="badge {tone}">{status}</span></header>{warning}{waiting}
<div class="cards"><div class="card"><span class="muted">当前阶段</span><strong>{escape(phase_label(phase))}</strong></div>
<div class="card"><span class="muted">相机到目标参考点的光学深度</span><strong>{depth_text}</strong></div>
<div class="card"><span class="muted">已运行仿真时间（分:秒）</span><strong>{duration(stamp)}</strong></div></div>
<div class="columns"><section class="panel"><h2>任务阶段</h2><ul class="steps">{''.join(steps)}</ul>
<p class="foot">这里显示程序的阶段检查；整段是否通过，以运行结束后的独立物理验收为准。</p></section>
<div><section class="panel"><h2>最近一组实采双目图像</h2><div class="pictures">{pictures}</div>
<p>{visual}</p><div class="muted">{freshness}</div>
<p class="foot">原始相机图像。远处尚未采样新照片时，采样时间会明确显示，不表示已经获得新观测。</p></section>
{final_checks}</div></div>
<footer class="foot">每2秒自动刷新 · 页面更新时间 {updated}（北京时间） · 仿真数据约 {age:.1f} 秒前更新<br>
V-bar负责服务星接近和保持；双目提供导航观测；机械臂执行最后抓取。</footer>
</main></body></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-directory', type=Path, required=True)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()
    directory = args.run_directory.resolve(strict=True)
    while True:
        html = render(directory)
        temporary = directory/'progress.tmp.html'
        temporary.write_text(html, encoding='utf-8')
        try:
            temporary.replace(directory/'progress.html')
        except PermissionError:
            pass  # A browser can briefly hold the previous local file open.
        if args.once or (directory/'acceptance.json').exists() or (directory/'failure.json').exists():
            break
        time.sleep(2)


if __name__ == '__main__':
    main()
