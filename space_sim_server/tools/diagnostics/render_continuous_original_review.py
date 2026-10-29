"""Make a report chart and review page from actual continuous trial samples."""
import argparse
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

from tools.diagnostics.audit_continuous_original_stereo import audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    result = audit(directory)
    rows = []
    for line in (directory/'trajectory.jsonl').read_text(encoding='utf-8').splitlines():
        try:rows.append(json.loads(line))
        except json.JSONDecodeError:break
    t = np.array([r['time_s'] for r in rows])
    font = FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True, constrained_layout=True)
    axes[0].plot(t,[r['true_optical_depth_m'] for r in rows],color='#1e78b7')
    axes[0].set_ylabel('真实光学深度 / m',fontproperties=font)
    axes[1].plot(t,[r['filtered_position_error_mm'] for r in rows],color='#d47b0b',label='导航估计减真值')
    axes[1].plot(t,[r['true_tracking_error_mm'] for r in rows],color='#38964c',alpha=.8,label='实际位置减当前参考')
    axes[1].set_ylabel('三维位置误差 / mm',fontproperties=font)
    axes[1].legend(prop=font,loc='upper right')
    angle = [np.nan if r['policy']['phase'] in {'NavigationAcquire','Station17Approach','Station17Hold'}
             else max(1e-5,r['true_target_reference_error_deg']) for r in rows]
    axes[2].semilogy(t,angle,color='#8b56ae')
    axes[2].axhline(1,color='#c24b4b',ls='--',lw=1)
    axes[2].set_ylabel('姿态跟踪误差 / °',fontproperties=font)
    axes[3].step(t,[int(r['visual_ready']) for r in rows],where='post',color='#209a95')
    axes[3].set_yticks([0,1],['未就绪','就绪'],fontproperties=font)
    axes[3].set_ylim(-.1,1.15)
    axes[3].set_ylabel('原双目视觉',fontproperties=font)
    axes[3].set_xlabel('同一次仿真的时间 / s',fontproperties=font)
    labels={'TargetPointing':'17.4 m调姿','GNSSBridge5p5Hold':'5.5 m停泊',
        'OriginalStereoAcquire':'近距视觉等待','VisualApproach':'视觉放行',
        'PlatformEntry':'平台进入','Grasp':'机械臂接续','Complete':'抓持完成','SafeHold':'停止保持'}
    for ax in axes:
        ax.grid(alpha=.2)
        for event in result['phases']:
            if event['phase'] in labels:ax.axvline(event['time_s'],color='#8998a4',alpha=.4,lw=.8)
    for n,event in enumerate(e for e in result['phases'] if e['phase'] in labels):
        axes[0].annotate(labels[event['phase']],xy=(event['time_s'],.98),xycoords=('data','axes fraction'),
            xytext=(5,-12-(n%3)*22),textcoords='offset points',fontproperties=font,fontsize=9,
            rotation=0,va='top')
    fig.suptitle('原相机连续接近诊断：GNSS衔接＋持续稳姿＋真实双目接入',fontproperties=font,fontsize=17)
    fig.savefig(directory/'continuous_timeline.png',dpi=160)
    plt.close(fig)
    table=[]
    node_names={'17.4m GNSS stop':'17.4 m导航停泊',
        '5.5m GNSS-only diagnostic stop':'5.5 m导航停泊（诊断）',
        'near original-camera visual acquisition':'1.5 m停稳与真实视觉联合检查',
        '0.5m platform entry':'工具间隙0.5 m进入检查',
        '0.5m platform residence':'工具间隙0.5 m驻留'}
    for node in result['nodes']:
        error=node.get('actual_fixed_station_position_error_mm')
        error='—' if error is None else f"{error['maximum']:.3f} mm"
        speed=node.get('maximum_sampled_speed_mm_s')
        speed='—' if speed is None else f'{speed:.3f} mm/s'
        nav=node.get('navigation_position_error_mm')
        nav='—' if nav is None else f"{nav['maximum']:.3f} mm"
        tool=node.get('actual_tool_position_error_mm')
        tool='—' if tool is None else f"{tool['maximum']:.3f} / {node['actual_tool_gap_error_mm']['maximum']:.3f} mm"
        status='分项通过' if node.get('position_pass') and node.get('speed_pass') else node['result']
        table.append('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in
            (node_names.get(node['node'],node['node']),error,speed,nav,tool,status))+'</tr>')
    completed=result.get('requested_stage_completed',False)
    finished=(directory/'summary.json').exists() or (directory/'failure.json').exists()
    status='已完成本次请求阶段' if completed else ('本轮已结束，未通过全部阶段' if finished else '正在运行，后续阶段待验证')
    failure=next((r['policy']['failure_reason'] for r in rows if r['policy'].get('failure_reason')),None)
    if failure is None:
        failure=result.get('run_failure',{}).get('reason')
    failure_note='' if failure is None else '<p>本轮停止原因：'+html.escape(failure)+'</p>'
    video=next((p for p in ('overview_sampling_caption.mp4','overview_full.mp4','overview_to_first_vision.mp4') if (directory/p).exists()),None)
    video_note=('' if video is None else '<h2>原始双目输入回放</h2><p>10倍播放；远处按较长的仿真时间间隔采集图像，间隔内重复显示同一张采样原图；这不表示传输丢帧。'
        '视频只覆盖其标注的仿真时间，不将独立初始化写成远距交接。</p><video controls preload="metadata" '
        'style="width:100%" src="'+video+'"></video>')
    page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>17.4 m之后的原相机连续试验</title><style>body{{font:17px/1.7 system-ui,"Microsoft YaHei",sans-serif;background:#edf2f6;color:#203248}}main{{max-width:1200px;margin:auto;padding:25px}}img{{width:100%}}table{{border-collapse:collapse;width:100%;background:white}}th,td{{border:1px solid #ccd5df;padding:10px}}.scope{{background:#fff1d0;padding:15px}}</style><main>
<h1>17.4 m之后：原相机连续试验</h1><p>{status}；最新仿真时间{result['last_time_s']:.2f} s，阶段{html.escape(result['last_phase'])}。</p>{failure_note}
<p class="scope">原100°双目、23块靶标与原灯光。此次显式使用独立GNSS衔接诊断配置，允许GNSS越过原12 m视觉保护，并把5.5 m检查限定为导航停泊；原网页保护不变。不能据此声称原方案或110 m起的完整任务已经通过。</p>
<p>顺序：17.4 m停稳 → 目标星首次调姿 → 接近时持续稳姿 → 5.5 m导航停泊 → 近处等待真实视觉 → 原双目参与导航 → 工具间隙0.5 m驻留 → 同一动力学状态交给机械臂。</p>
<img src="continuous_timeline.png" alt="真实距离、位置误差、姿态跟踪及视觉就绪时间线">
<table><tr><th>节点</th><th>实际站位偏差最大值</th><th>采样速度最大值</th><th>导航估计误差最大值</th><th>TCP位置 / 间隙偏差最大值</th><th>本次状态</th></tr>{''.join(table)}</table>
<p>实际位置偏差、导航估计误差分别统计；速度由20 Hz真值位置差分复算。图像{result['image_pairs']}组，其中{result['image_pairs_accepted']}组进入滤波。<a href="independent_audit.json">逐帧审计结果</a></p>
{video_note}
<p><a href="../../../docs/FROM_17P4M_CONTINUOUS_ORIGINAL_STEREO_20261029.md">17.4 m之后的流程说明</a> · <a href="../../../docs/TARGET_MARKER_REPORT_20261029.md">靶标信息与截图</a></p></main></html>'''
    (directory/'index.html').write_text(page,encoding='utf-8')
    (directory/'independent_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(review=str(directory/'index.html'),status=status,last_sim_s=result['last_time_s']),ensure_ascii=False))


if __name__=='__main__':
    main()
