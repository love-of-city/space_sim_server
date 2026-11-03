"""HTML review of unmodified real-platform image evidence, no synthetic views."""
import argparse
import html
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    entries = json.loads((root/'run/imported_scene_library.json').read_text(encoding='utf-8-sig'))['entries']
    sections = []
    verified = 0
    image_count = 0
    for entry in entries:
        relative = entry['id'].removeprefix('library/')
        directory = args.input/relative
        report_path = directory/'inspection.json'
        if not report_path.exists():
            sections.append(f'<section><h2>{html.escape(entry["label"])}</h2><p>尚未取得完整截图。</p></section>')
            continue
        report = json.loads(report_path.read_text(encoding='utf-8'))
        complete = (directory/'complete.json').exists()
        verified += complete
        figures = []
        def figure(filename, caption):
            nonlocal image_count
            image_count += 1
            url = html.escape(relative+'/'+filename, quote=True)
            return f'<figure><a href="{url}" target="_blank"><img loading="lazy" src="{url}" alt="{html.escape(caption)}"></a><figcaption>{html.escape(caption)}</figcaption></figure>'
        primary = next((s for s in report['streams'] if s['id']=='BskRenderer' and s.get('available')),None)
        if primary:
            figures.append(figure(primary['workbench'],'实际网页工作台'))
        for stream in report['streams']:
            if stream.get('available'):
                figures.append(figure(stream['image'], stream['label']+' · '+stream['id']))
        for item in ([] if entry['base_template_id']=='sarm-dual-capture26' else report.get('vision_images',[])):
            figures.append(figure(item['file'],'双目 '+item['eye']+' · 已匹配当前渲染会话'))
        instance = report['state'].get('scene_runtime',{}).get('instance') or {}
        observation = report['state'].get('simulation',{}).get('latest_observation') or {}
        dual = entry['base_template_id']=='sarm-dual-capture26'
        note = ('调整对接方位前的基线截图：原总览主要覆盖操作星。第 2、4 类从已对接状态开始，不评审对接双目；重点检查总览和作业腕部视角。'
                if dual else '本星五项共用同一任务盒与初始作业姿态，任务配方不同，因此开始画面相近。相机安装位姿与视场角沿用原系统。')
        if entry['kind']=='rendezvous':
            note='实际飞行系统初段截图。双星距离较大时，总览中的模型很小属于真实比例；本次截图不重复完整交会验收。'
        provenance = html.escape(str(instance.get('instance_id') or report['state'].get('mission',{}).get('mission_id')))
        sections.append(f'<section id="{html.escape(relative.replace("/","-"))}"><h2>{html.escape(entry["label"])}</h2><p>{note}</p><p class="meta">系统加载与截图：{"完成" if complete else "待补全"} · 实例 {provenance} · 仿真时间 {html.escape(str(observation.get("sim_time_ns","见原始状态")))} ns · <a href="{html.escape(relative)}/inspection.json">状态与截图来源</a> · <a href="{html.escape(relative)}/camera-configuration.json">生效相机配置（操作场景）</a></p><div class="grid">'+''.join(figures)+'</div></section>')
    style='body{font-family:Segoe UI,Microsoft YaHei,sans-serif;background:#101826;color:#e7edf5;max-width:1500px;margin:30px auto;padding:0 24px}h1{font-size:28px}h2{font-size:21px}p{line-height:1.7}a{color:#89bfff}.meta{font-size:12px;color:#a8b8ca}section{border-top:1px solid #344257;padding:20px 0}.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px}figure{margin:0;background:#1b283b;padding:8px;border-radius:8px}img{width:100%;height:260px;object-fit:contain;background:#080d16}figcaption{font-size:12px;overflow-wrap:anywhere;padding-top:8px}@media(max-width:900px){.grid{grid-template-columns:1fr 1fr}}'
    intro=f'<h1>前四类场景 · 正式仿真系统视角检查</h1><p>已完成 {verified}/14 个场景，{image_count} 张展示图。图片来自现有 Space Arm 网页工作台与 UE 实时视频/双目图像接口。点击图片查看原始分辨率。</p><p>保持原系统相机安装位置、朝向和视场角。系统加载成功与视角满足作业要求分别判断；以下图片保留遮挡、逆光和覆盖不足。独立 UE 回放截图不在本报告中。任务完整动力学验收另见已验收库，本次是实际导入和相机检查。</p><p><a href="http://127.0.0.1:8000" target="_blank">打开本地仿真工作台</a> · 在“场景模板”选择同名“场景库 · ”入口，再启动场景。切换任务前停止当前场景。</p>'
    (args.input/'index.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>场景库实际系统截图</title><style>'+style+'</style><body>'+intro+''.join(sections)+'</body></html>',encoding='utf-8')
    print(json.dumps({'complete':verified,'total':14,'displayed_images':image_count,'gallery':str(args.input/'index.html')},ensure_ascii=False))


if __name__=='__main__':
    main()
