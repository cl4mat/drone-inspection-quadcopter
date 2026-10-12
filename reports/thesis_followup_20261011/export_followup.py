#!/usr/bin/env python3
"""Export additional thesis evidence; missing trials stay explicitly missing."""
from pathlib import Path
import json, shutil, hashlib, csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from docx import Document
from docx.shared import Inches,Pt,Cm
from docx.oxml.ns import qn
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,Image
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from fontTools.ttLib import TTFont as FontMap
from xml.sax.saxutils import escape

P=Path(__file__).resolve().parents[2]
ICP=P/'reports/icp_registration_20261011'
DYN=P/'reports/dynamic_obstacle_margin05_20261011'
BASE=P/'reports/dynamic_obstacle_formal_prior_20261011'
QT=P/'reports/qt_response_20261011'
OUT=Path.home()/'Desktop'/'终期答辩_追加实验_20261011'
for sub in ['', '图片','原始数据','来源记录']:(OUT/sub).mkdir(parents=True,exist_ok=True)
items=[]
def h(t):items.append(('h',t))
def p(t):items.append(('p',t))
def table(headers,rows):items.append(('t',(headers,rows)))
def img(source,name,caption):
    if source.exists():
        shutil.copy2(source,OUT/'图片'/name);items.append(('i',(name,caption)))

h('终期答辩追加实验：场景、地图、动态避障和Qt响应')
p('统计日期：2026年10月11日。本文件补充上一版报告未测量的项目，保留数据来源与测量定义。未完成的项目明确列出，不将准备阶段故障写成正式试验成功。原答辩文档保持原样。')
h('1. 四类图片')
img(ICP/'shot_gazebo_building.png','图1_Gazebo双层楼道截图.png','图1：实际Gazebo界面中的双层楼道场景（其他AI保存的真实截图）。')
img(ICP/'fig_point_cloud_map.png','图2_存档体素化点云地图.png','图2：SLAM占据存档的体素化点云地图（0.10 m占据体素中心点；不是原始Faster-LIO PCD）。')
img(ICP/'fig_voxel_map.png','图3_体素地图.png','图3：对应的0.10 m占据体素地图。')
if (QT/'shot_qt_online.png').exists():
    img(QT/'shot_qt_online.png','图4_Qt在线界面截图.png','图4：本轮仿真实测中的Qt操作台真实在线截图。')
else:
    img(ICP/'shot_qt_preview.png','图4_Qt预览截图.png','图4：实际Qt地图预览界面；截图时飞控未运行，红色状态卡不能解释为完整飞行系统健康。在线截图尚待采集。')
h('2. ICP配准与点云重合度')
r=json.loads((ICP/'icp_overlap_result.json').read_text())
shutil.copy2(ICP/'icp_overlap_result.json',OUT/'原始数据/ICP结果.json')
p('将389403个SLAM存档占据体素中心点转换到仿真世界坐标，与实际双层SDF模型的98个碰撞盒表面进行点到点ICP配准。真值表面采样2812792点，采样间距0.04 m；ICP固定随机抽样40000个地图点，最大对应距离0.12 m，保留匹配内较近的80%对应点，最多60次迭代。最终重合度使用全部地图点到真实有向盒表面的解析距离评价，避免把有限表面采样间距当成测量误差。')
p('重合度定义：地图点到最近真值盒面距离≤τ的点数/全部地图点数。主阈值τ=0.10 m与地图分辨率一致，补充报告0.05、0.15和0.20 m阈值；这是单向地图几何一致性，不是整个场景表面覆盖率或巡检相机覆盖率。')
table(['阈值/m','配准前/%','配准后/%'],[[t,f"{r['before_icp']['overlap_pct'][t]:.2f}",f"{r['after_icp']['overlap_pct'][t]:.2f}"] for t in ['0.05','0.1','0.15','0.2']])
p(f"可直接粘贴：对SLAM占据存档的体素化点云进行ICP配准后，在0.10 m容差下，点云与Gazebo真值模型的重合占比为{r['after_icp']['overlap_pct']['0.1']:.2f}%；在0.20 m容差下为{r['after_icp']['overlap_pct']['0.2']:.2f}%。全部地图点到真值表面的距离RMSE为{r['after_icp']['distance_m']['rmse']:.4f} m，ICP修正平移量为{r['after_icp']['icp_correction_translation_norm_m']:.4f} m、旋转角为{r['after_icp']['icp_correction_rotation_deg']:.4f}°。原始Faster-LIO全局PCD未归档，本项测量对象是0.10 m占据存档的体素中心点。")
p('核对其他AI脚本后修复了ICP重复应用累积变换、倾斜盒距离未先减中心再旋转的两处计算问题。旧结果84.37%/98.05%已弃用，旧计算副本不纳入仓库；本目录只保留修正后的最终数据。')
img(ICP/'icp_overlap.png','ICP重合度与收敛.png','ICP重合度随阈值变化、匹配内对应点RMSE收敛曲线；收敛曲线的匹配内RMSE与全地图RMSE口径不同。')
img(ICP/'icp_registration_overlay.png','ICP配准叠加图.png','ICP配准后存档体素化点云与Gazebo真值盒模型叠加。')
h('3. 动态避障组数、成功数与成功率')
if (BASE/'groups/group_01/group_evaluated.json').exists():
    baseline=json.loads((BASE/'groups/group_01/group_evaluated.json').read_text())
    p(f"默认0.40 m水平膨胀配置曾记录1组有效试验：最终到点误差{baseline['final_goal_error']:.4f} m并返回原路线，但0.40 m保守整机包络最小间距{baseline['min_clearance']:.4f} m，严格判为包络不合格，不能计避障成功。到点后的事件记录存在重复sim字段异常，评分已依据同一原始轨迹、感知事件及rosbag原目标恢复；原错误记录保留。该组和下述0.50 m配置分别统计，不混写成同一配置性能。")
    for name in ['group.json','group_evaluated.json','ground_truth.jsonl','recovery.log','astar_diagnosis.json']:
        src=BASE/'groups/group_01'/name
        if src.exists():
            dest=OUT/'原始数据/默认配置基线';dest.mkdir(exist_ok=True);shutil.copy2(src,dest/name)
if (DYN/'summary.json').exists():
    data=json.loads((DYN/'summary.json').read_text());groups=data.get('groups',[])
    valid=[g for g in groups if g.get('code')!='F7_INFRASTRUCTURE']
    success=[g for g in valid if g.get('perception_hits',0)>0 and g.get('min_clearance') is not None and g['min_clearance']>0]
    goal_success=[g for g in valid if g.get('code')=='SUCCESS']
    rate=100*len(success)/len(valid) if valid else None
    goal_rate=100*len(goal_success)/len(valid) if valid else None
    p(f'本轮正式已记录{len(groups)}组，有效障碍暴露N={len(valid)}组；10组均有障碍感知证据且整机包络间距为正，因此无碰撞避障成功M={len(success)}，成功率{rate:.2f}%。其中到达目标并完成回归{len(goal_success)}/{len(valid)}组（{goal_rate:.1f}）。')
    if success:
        errors=[g['final_goal_error'] for g in goal_success]
        p(f'到达目标的{len(goal_success)}组真实终点误差均值{np.mean(errors):.4f} m、最大{max(errors):.4f} m；10组最小整机包络间距为{min(g["min_clearance"] for g in success):.4f} m。第7组虽未到达目标，但飞机保持悬停并以1.2374m最小包络间距结束，无碰撞，按本报告避障判据计成功。')
    p('实验定义：加载实测静态地图，在无人机已经沿原目标路线移动后，生成一块此前不存在的静态箱体，测试在线感知和反应式重规划。它不等价于具备移动障碍运动预测的测试。目标保持不变；规划地图感知和全程安全包络间距用于“避障是否成功”，真实目标到达与三维路线回归另外统计。本组配置使用0.50 m水平膨胀、0.5 m/s默认移动速度、1.2 m相对起飞高度；注入箱体0.30×2.00×1.80 m，距机体前方2.2 m，三个目标横向偏移−1.5/0/+1.5 m。按用户要求缩短后续航程，第1–4组前向目标距离5.5 m，第5–10组约4.2 m；第8–10组采用固定FCU终点，起点到达误差使实际前向距离为4.25–4.29 m；第1–5组分别冷启动，第6–7组连续飞行，第7组返向时EGO因候选局部轨迹触及膨胀障碍安全边界而拒绝发布轨迹，人工结束导航并安全HOLD：无碰撞，计入避障成功，但目标未完成；组间近零角转向的脚本异常随后触发降落；修复后第8–10组复用原仿真连续飞行，空路线返回固定FCU起点再做正向障碍暴露，组间用真实雷达清除箱体占据证据，最后统一降落；第6组起READY连续稳定等待为1墙秒（此前3秒）。逐组parameters.json保留配置，两种重复方式分别注明，不把连续组称为独立冷启动。试验位置在双层场景的一楼大厅，不代表楼梯上的动态避障也已专项验证。')
    p('本报告的避障成功判据：出现障碍占据体素感知证据，且记录中的整机0.40 m保守球形包络到障碍最小距离始终大于0。目标到达与回归原路径是独立导航指标：10组中9组完成并回归原路径；group_07前方规划位置存在风险，EGO判定局部候选曲线与膨胀障碍冲突而拒绝发布轨迹，飞机安全HOLD，未到达目标但没有碰撞，按无碰撞口径计避障成功。正包络间距是保守几何无碰撞核验，不等同于接触传感器测量。')
    table(['组','避障结果','导航结果','回归原路径','最小包络间距/m','到点误差/m'],[[g.get('group'),'无碰撞成功' if g.get('perception_hits',0)>0 and g.get('min_clearance',-1)>0 else '包络侵入/无证据','到达并回归' if g.get('code')=='SUCCESS' else 'EGO拒绝规划，未到点' if g.get('group')=='group_07' else str(g.get('code')),'是' if json.loads((DYN/'groups'/g['group']/'group.json').read_text()).get('returned_to_route') else '否',str(g.get('min_clearance')),str(g.get('final_goal_error'))] for g in groups])
    stats = []
    for g in valid:
        src = DYN/'groups'/g['group']/'group.json'
        detail = json.loads(src.read_text()) if src.exists() else {}
        events = detail.get('events', [])
        times = {e['event']: e['sim'] for e in events if e.get('sim') is not None}
        parameters=json.loads((src.parent/'parameters.json').read_text()) if (src.parent/'parameters.json').exists() else {}
        stats.append(dict(group=g['group'], forward_m=parameters.get('forward'), lateral=g.get('lateral'), code=g.get('code'), collision_free=bool(g.get('perception_hits',0)>0 and g.get('min_clearance') is not None and g['min_clearance']>0), goal_completed=g.get('code')=='SUCCESS', returned_to_route=bool(detail.get('returned_to_route')),
                          min_clearance_m=g.get('min_clearance'), goal_error_m=g.get('final_goal_error'),
                          navigation_sim_s=(times['arrived']-times['goal_issued']) if 'arrived' in times and 'goal_issued' in times else None,
                          perception_sim_s=(times['perceived']-times['spawn_confirmed']) if 'perceived' in times and 'spawn_confirmed' in times else None))
    if stats:
        with (OUT/'原始数据/动态指标明细.csv').open('w', newline='') as handle:
            w=csv.DictWriter(handle, fieldnames=list(stats[0]));w.writeheader();w.writerows(stats)
        table(['组','无碰撞','目标到达','回归原路径','导航耗时/仿真秒','感知延迟/仿真秒'],[[g['group'],'是' if g['collision_free'] else '否','是' if g['goal_completed'] else '否','是' if g['returned_to_route'] else '否',round(g['navigation_sim_s'],3) if g['navigation_sim_s'] is not None else '未到达',round(g['perception_sim_s'],3) if g['perception_sim_s'] is not None else '无证据'] for g in stats])
        p('导航耗时从发布原目标到COMPLETED；group_07未到点，不填写导航到达耗时。不含启动、起飞、组间清障和空路线返程；results.csv中的wall_seconds是组内脚本墙钟运行时间，冷启动组包含准备与起降，连续组只含当组执行，不用于两种方式的直接效率比较；感知延迟从箱体生成确认到首条有效新增占据证据，二者均为仿真时钟。仿真约0.25倍实时运行，不把这些数值当作实机墙钟性能。十组为有限次数的本场景重复测试，不能由样本成功率推断任意场景都能成功。第7组在障碍已感知、EGO持续拒绝危险局部轨迹后结束；无碰撞按避障成功计数，未到目标仍记为导航未完成。')
        plt.rcParams['font.family']=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
        fig,axes=plt.subplots(1,2,figsize=(12,4.5))
        labels=[g['group'].replace('group_','') for g in stats]
        axes[0].bar(labels,[g['min_clearance_m'] if g['min_clearance_m'] is not None else np.nan for g in stats],color='#38936e')
        axes[0].axhline(0,color='#c35755',ls='--');axes[0].set(title='最小整机包络间距',xlabel='实验组',ylabel='米')
        axes[1].bar(labels,[g['goal_error_m'] if g['goal_error_m'] is not None else np.nan for g in stats],color='#517fab')
        axes[1].axhline(.30,color='#c35755',ls='--',label='到达评价上限');axes[1].set(title='实际终点误差',xlabel='实验组',ylabel='米');axes[1].legend()
        fig.tight_layout();fig.savefig(OUT/'图片/动态避障安全间距与终点误差.png',dpi=240);plt.close(fig)
        items.append(('i',('动态避障安全间距与终点误差.png','逐组真值评估：整机包络间距与终点误差。')))
    for name in ['summary.json','summary.json.navigation_completion_original.json','metadata.json','results.csv','raw_data_audit.json']:
        if (DYN/name).exists():shutil.copy2(DYN/name,OUT/'原始数据'/('动态_'+name))
    for group in (DYN/'groups').glob('*'):
        folder=OUT/'原始数据/动态逐组'/group.name;folder.mkdir(parents=True,exist_ok=True)
        for name in ['group.json','ground_truth.jsonl','samples.jsonl','driver.log','group_evaluated.json','parameters.json','between_groups_clearance.json','operator_abort.json','preparation_events.json']:
            if (group/name).exists():shutil.copy2(group/name,folder/name)
    plt.rcParams['font.family']=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
    fig,ax=plt.subplots(figsize=(9,4.5))
    goal_only=sum(g['code']=='SUCCESS' and g.get('perception_hits',0)>0 and g.get('min_clearance',-1)>0 for g in valid)
    no_goal=len(success)-goal_only
    ax.bar(['无碰撞且到达目标','无碰撞但未到点','包络侵入/失效'],[goal_only,no_goal,len(groups)-len(valid)],color=['#38936e','#719f88','#c35755'])
    ax.set(ylabel='组数',title=f'动态避障结果：无碰撞 {len(success)}/{len(valid)}（{rate:.0f}%）');ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True));
    for patch in ax.patches:ax.text(patch.get_x()+patch.get_width()/2,patch.get_height()+.1,str(int(patch.get_height())),ha='center');
    fig.tight_layout();fig.savefig(OUT/'图片/动态避障结果统计.png',dpi=240);plt.close(fig)
    items.append(('i',('动态避障结果统计.png','按用户确认的无碰撞判据计数；另列未到导航目标但安全悬停的组。')))
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for group in (DYN/'groups').glob('*'):
        datafile=group/'ground_truth.jsonl';resultfile=group/'group.json'
        if not datafile.exists() or not resultfile.exists():continue
        record=json.loads(resultfile.read_text());coords=[]
        for line in datafile.read_text().splitlines():
            row=json.loads(line)
            if row.get('world') and row.get('flight_state')=='NAVIGATING':coords.append(row['world'])
        if not coords:continue
        xyz=np.asarray(coords)
        axes[0].plot(xyz[:,0],xyz[:,1],label=group.name,lw=1)
        axes[1].plot(xyz[:,0],xyz[:,2],label=group.name,lw=1)
    axes[0].set(xlabel='世界X/m',ylabel='世界Y/m',title='实际水平绕行轨迹');axes[0].axis('equal');axes[0].legend()
    axes[1].set(xlabel='世界X/m',ylabel='世界Z/m',title='实际飞行高度变化');axes[1].legend()
    fig.tight_layout();fig.savefig(OUT/'图片/动态避障实际轨迹.png',dpi=240);plt.close(fig)
    items.append(('i',('动态避障实际轨迹.png','真值评估侧记录的实际运动轨迹，不是独立生成的规划示意线。')))
else:p('正式动态避障测试尚未完成，没有可填入的试验组数与成功率。准备阶段的短航行、地图加载或心跳故障不计为障碍暴露成功。')
h('4. Qt状态回传与悬停接管响应')
if (QT/'summary.json').exists():
    q=json.loads((QT/'summary.json').read_text())
    table(['项目','样本数','P95/ms','最大值/ms'],[[label,str(q[key]['n']),f"{q[key].get('p95',0):.2f}",f"{q[key].get('max',0):.2f}"] for label,key in [('健康状态发布至Qt回调','health_receive_ms'),('健康状态发布至界面更新','health_display_ms'),('飞行中状态发布至界面更新','airborne_health_display_ms'),('按钮触发至服务确认','hold_service_ack_ms'),('按钮触发至HOLD状态接收','hold_state_receipt_ms'),('按钮触发至HOLD界面更新','hold_display_ms')]])
    p(f"本轮实际Qt悬停按钮试验{q['trials']}次，成功计时记录{q['successful_hold_rows']}次。使用同一主机的CLOCK_MONOTONIC墙钟，避免将低实时因子的仿真秒误写为真实毫秒。状态时延从飞行管理器实际健康快照发布计至Qt接收和状态栏更新；悬停响应从真实Qt按钮处理函数开始，计至服务ACK、HOLD回传与界面更新。它不包含鼠标硬件延迟、传感器采集至LIO输出的时间，也不等于机体完全停止运动时间。")
    p('报告建议同时写P95和本次观测最大值，表述“本次仿真中观测到的最大延迟”，不要把样本最大值写成任意网络、实机或长期运行下的保证。150 ms只是界面定时刷新配置，不是端到端实测数据。')
    for name in ['summary.json','qt_timing.csv','command_trials.json']:
        if (QT/name).exists():shutil.copy2(QT/name,OUT/'原始数据'/('Qt_'+name))
else:p('按用户本轮要求，Qt专项计时暂缓，优先增加避障重复试验。已经增加实际状态快照发布、Qt接收、界面刷新和实际悬停按钮服务确认/HOLD回传计时，但实现计时代码不等于获得测量结果。目前不填入毫秒数。')
h('5. 数据限制与来源')
p('所有结果属于Gazebo/PX4仿真。真值只用于障碍放置及离线评价，不进入定位、EKF或导航控制。原始全局PCD无法从占据存档恢复为原扫描数据，因此图2和ICP对象必须注明体素化存档。准备失败资料保留在项目reports/dynamic_obstacle_feasibility*目录；正式试验失败不能从统计中删除。')
p('源目录：reports/icp_registration_20261011；reports/dynamic_obstacle_margin05_20261011（正式重复组）；reports/dynamic_obstacle_formal_prior_20261011（默认配置失败基线）；reports/qt_response_20261011（暂缓测量）。图片、逐组原始记录、已完成项目的CSV和统计JSON随本文打包（Qt时延未测量，不包含时延CSV）；上一版定位、80目标导航和98.57%巡检覆盖率数据保持原口径。')

doc=Document();sec=doc.sections[0];sec.left_margin=sec.right_margin=Cm(2)
for name in ['Normal','Heading 1']:
    style=doc.styles[name];style.font.name='微软雅黑';style._element.rPr.rFonts.set(qn('w:eastAsia'),'微软雅黑')
doc.styles['Normal'].font.size=Pt(10.5)
cjk='/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf';cmap=FontMap(cjk).getBestCmap()
pdfmetrics.registerFont(TTFont('CJK',cjk));pdfmetrics.registerFont(TTFont('Latin','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
body=ParagraphStyle('body',fontName='CJK',fontSize=10,leading=16,wordWrap='CJK',spaceAfter=9)
heading=ParagraphStyle('head',parent=body,fontSize=15,leading=22,spaceBefore=12)
small=ParagraphStyle('small',parent=body,fontSize=8,leading=12)
def rich(t):
    parts=[]
    for c in str(t):
        font='CJK' if ord(c) in cmap else 'Latin'
        if parts and parts[-1][0]==font:parts[-1][1]+=c
        else:parts.append([font,c])
    return ''.join(f'<font name="{f}">{escape(v)}</font>' for f,v in parts)
story=[];md=[]
for kind,value in items:
    if kind=='h':doc.add_heading(value,1);story.append(Paragraph(rich(value),heading));md+=['## '+value,'']
    elif kind=='p':doc.add_paragraph(value);story.append(Paragraph(rich(value),body));md+=[value,'']
    elif kind=='t':
        headers,rows=value;t=doc.add_table(rows=1,cols=len(headers));t.style='Light Shading Accent 1'
        for c,v in zip(t.rows[0].cells,headers):c.text=str(v)
        for row in rows:
            for c,v in zip(t.add_row().cells,row):c.text=str(v)
        tab=Table([[Paragraph(rich(v),small) for v in row] for row in [headers]+rows],colWidths=[(A4[0]-100)/len(headers)]*len(headers),repeatRows=1)
        tab.setStyle(TableStyle([('GRID',(0,0),(-1,-1),.5,colors.lightgrey),('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e4edf4')),('VALIGN',(0,0),(-1,-1),'TOP')]))
        story.extend([tab,Spacer(1,12)]);md+=['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|']+['|'+'|'.join(map(str,row))+'|' for row in rows]+['']
    else:
        name,caption=value;doc.add_picture(str(OUT/'图片'/name),width=Inches(6.3));doc.add_paragraph(caption)
        from PIL import Image as PILImage
        with PILImage.open(OUT/'图片'/name) as im:ratio=im.height/im.width
        story.extend([Image(str(OUT/'图片'/name),width=A4[0]-100,height=(A4[0]-100)*ratio),Paragraph(rich(caption),small),Spacer(1,12)])
        md+=['!['+caption+'](图片/'+name+')',caption,'']
doc.save(OUT/'终期答辩_追加实验.docx')
SimpleDocTemplate(str(OUT/'终期答辩_追加实验.pdf'),pagesize=A4,leftMargin=50,rightMargin=50,topMargin=40,bottomMargin=45).build(story)
(OUT/'终期答辩_追加实验.md').write_text('\n'.join(md))
sources=[ICP/'icp_overlap_result.json',ICP/'run_icp_overlap.py']+[x for x in [DYN/'summary.json',QT/'summary.json'] if x.exists()]
(OUT/'来源记录/文件SHA256.json').write_text(json.dumps([dict(path=str(x),sha256=hashlib.sha256(x.read_bytes()).hexdigest()) for x in sources],ensure_ascii=False,indent=2))
shutil.make_archive(str(OUT),'zip',root_dir=OUT.parent,base_dir=OUT.name)
print(str(OUT),flush=True)
