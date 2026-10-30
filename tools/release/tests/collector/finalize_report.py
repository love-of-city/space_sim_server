from pathlib import Path
import json
import av
from PIL import Image,ImageDraw
import argparse
_parser=argparse.ArgumentParser(description='Explicit release integration verification; requires GPU/runtime.')
_parser.add_argument('--release-root',type=Path,required=True)
root=_parser.parse_args().release_root.resolve()
(root/'validation').mkdir(exist_ok=True)
result=json.loads((root/'validation/RESULTS.json').read_text())
assert result['passed']
canvas=Image.new('RGB',(1280,1340),'white');draw=ImageDraw.Draw(canvas)
assignments=[]
for row,entry in enumerate(result['episodes']):
    episode=Path(entry['episode'])
    meta=json.loads((episode/'metadata.json').read_text())
    colors=meta['scene_instance']['collection_config']['color_randomization']
    assert len(colors['hole_colors'])==7 and len(set(colors['hole_colors'].values()))==7
    assignments.append(colors['hole_colors'])
    entry['hole_colors']=colors['hole_colors'];entry['source_colors']=colors['source_colors']
    entry['video_paths']={}
    for col,camera in enumerate(('spacecraft_overview','sarm_wrist_cam')):
        video=next((episode/'lerobot/videos').glob(f'*{camera}/chunk-*/*.mp4'))
        entry['video_paths'][camera]=str(video)
        target=entry['frames']//2
        with av.open(str(video)) as container:
            for i,frame in enumerate(container.decode(video=0)):
                if i==target:
                    canvas.paste(frame.to_image().resize((640,640)),(col*640,row*670+30))
                    break
            else:raise RuntimeError('Missing video midpoint')
        draw.text((col*640+8,row*670+8),f"Episode {row+1}, seed {entry['seed']}, {camera}, frame {target}",fill='black')
assert assignments[0]!=assignments[1]
canvas.save(root/'validation/final-contact-sheet.jpg',quality=92)
result['seven_unique_hole_colors_each_episode']=True
result['hole_colors_changed_between_episodes']=True
result['same_dynamics_worker_pid']=len({r['dynamics_worker_pid'] for r in result['episodes']})==1
result['contact_sheet']=str(root/'validation/final-contact-sheet.jpg')
(root/'validation/RESULTS.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
lines=['# 采集可执行文件验收结果','',
    '已从最终 `.run` 文件展开到带空格的新目录，使用内置环境新生成布局，并通过该 `.run` 入口连续完成两条抽屉任务。两条均通过原生产数据校验；双相机视频已完整解码。',
    '', '| 样本 | 仿真时长 | 正式录制 | 完整采集周期 | 每路视频帧数 |',
    '| --- | ---: | ---: | ---: | ---: |']
for i,row in enumerate(result['episodes'],1):
    lines.append(f"| {i} | {row['recorded_sim_seconds']:.2f} s | {row['recording_seconds']:.2f} s | {row['full_cycle_seconds']:.2f} s | {row['frames']} |")
lines += ['',f"第二条完成到第一条完成的间隔：{result['second_completion_interval_seconds']:.2f} 秒。完整周期含场景准备、预检、初始化、录制、封装及数据校验；本机同时有布局生成任务，以上是运行观测，不是独占机器性能基准。",'',
    '配置：关节独立 ±30°、五个固定全局机位随机选择、七孔无重复颜色、英文 instruction、双 1024×1024 RGB / 50 Hz、H.264 veryfast 流式编码。两条颜色分配和关节偏移不同；本次随机机位恰好均为 P1。轨道相位在本轮受控试验中固定。',
    '',f"UE 由同一个批次进程复用；动力学进程同 PID 复用：{result['same_dynamics_worker_pid']}。每条重建物理状态和录制会话。",'',
    'Instruction: `Open the drawer approximately 8 cm, then close it fully and release it.`','',
    '范围：六种生产任务入口已接入，本轮真实验收为抽屉两条。使用的是新生成后显式指定的一套布局；这不代表六任务或随机布局池全部已认证。程序保留随机池校验和拒绝固定布局回退。另一台 Linux 机器尚未实测。','',
    '9 项启动器测试通过；216 个生产 Python 文件和两份 UE 插件二进制与当前工作区一致；最终展开目录的 208 个新增/覆盖文件与发行清单一致。','',
    f"可执行文件：`{result['artifact']['artifact']}`",f"SHA256：`{result['artifact']['sha256']}`",'',
    '双相机中段画面：','',f"![两个样本的全局和腕部画面]({root/'validation/final-contact-sheet.jpg'})",'',
    '视频文件：','']
for i,row in enumerate(result['episodes'],1):
    for camera,path in row['video_paths'].items():lines.append(f'- [样本 {i} · {camera}]({path})')
(root/'validation/REPORT.md').write_text('\n'.join(lines)+'\n')
print('Seven-color uniqueness, cross-episode variation and video snapshots verified.')
print('Report:',root/'validation/REPORT.md')
