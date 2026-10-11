# 데이터 수집기의 기본 랜덤화 범위를 포트폴리오 그래프로 정리한다.
# 코드에서 확인한 기본값을 사용하며 출력은 같은 폴더의 PNG, SVG, HTML이다.
# 확률은 min_card_count=1의 이론값이며 실제 수집 빈도가 아니다.
# 위치 단위는 항목별 m 또는 mm, 각도는 표시에 명시한 단위를 사용한다.
# 실행 인자 없음. MPLCONFIGDIR은 matplotlib 임시 캐시 위치를 지정한다.
from pathlib import Path
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'ws_aic/src/phy/phy_data_collection/phy_data_collection/runner/constants.py'
# 경로 계산 모듈을 실행하지 않고 상수 리터럴만 읽는다.
import ast
tree = ast.parse(SOURCE.read_text())
limits = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'LIMITS' for t in n.targets))
# 원본 범위가 바뀌면 고정 표를 조용히 재사용하지 않는다.
assert limits['nic_translation'] == (-0.0215, 0.0234)
assert limits['sc_translation'] == (-0.06, 0.055)
OUT = Path(__file__).resolve().parent
font = font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family': font.get_name(), 'font.size': 12, 'svg.fonttype': 'path', 'axes.unicode_minus': False})
ink, muted, orange, blue = '#2d3142', '#4f5d75', '#eb6c36', '#2e5aa8'
fig = plt.figure(figsize=(18,11.6), facecolor='#f5f5f5')
gs = fig.add_gridspec(2,3,left=.045,right=.975,bottom=.12,top=.855,wspace=.20,hspace=.36)
fig.text(.045,.946,'Randomizable Dataset Generator',fontsize=29,weight='bold',color=ink)
fig.text(.045,.909,'무엇이 바뀌는가?  |  장착 구성 · 목표 · 보드 · 로봇/케이블 · 촬영 자세 · 조명',fontsize=16,color=muted)

def panel(i,title,subtitle):
    ax=fig.add_subplot(gs[i//3,i%3]); ax.set_axis_off()
    ax.text(0,1.05,title,transform=ax.transAxes,fontsize=18,weight='bold',color=ink)
    ax.text(0,.955,subtitle,transform=ax.transAxes,fontsize=11,color=muted)
    return ax

def lines(ax,rows,y=.80,step=.13):
    for row in rows:
        ax.text(0,y,row,transform=ax.transAxes,fontsize=12,color=ink,va='top'); y-=step

a=panel(0,'01  카드 수 · 장착 레일','비어 있지 않은 장착 조합을 균등 추첨')
b=a.inset_axes([.08,.26,.89,.56]); p=[math.comb(5,k)/31*100 for k in range(1,6)]
assert abs(sum(p)-100)<1e-9
b.bar(range(1,6),p,color=['#cad1dc']*5,edgecolor=muted,width=.7)
for k,v in enumerate(p,1): b.text(k,v+1,f'{v:.1f}%',ha='center',fontsize=11,color=ink)
b.set(ylim=(0,40),xticks=range(1,6),yticks=[0,20,40],xlabel='SFP NIC 카드 수 (장)',ylabel='확률 (%)')
b.spines[['top','right']].set_visible(False); b.set_facecolor('#f5f5f5')
lines(a,['SFP: 5개 레일 → 31조합, 각 1/31','SC: 2개 레일 → 3조합, 각 1/3'],y=.10,step=.095)

a=panel(1,'02  목표 카드 · 포트','SFP / SC는 CLI에서 직접 선택 — 50:50 아님')
lines(a,['① 생성된 카드 중 목표 카드 선택','     활성 카드 K장 → 카드당 확률 1/K','② SFP: 카드의 포트 0 / 1 중 선택','     각 50%  ·  SC: sc_port_base 고정'],step=.16)
a.text(0,.09,'레일 인덱스: SFP 0–4 / SC 0–1\n빈 레일은 목표로 선택하지 않음',transform=a.transAxes,color=blue,fontsize=12)

a=panel(2,'03  보드 · 레일 내 위치','각 범위에서 균등 분포 (Uniform)')
rows=[['항목','SFP','SC'],['보드 X (m)','0.13 ~ 0.17','0.15 ~ 0.19'],['보드 Y (m)','−0.25 ~ −0.20','−0.05 ~ 0.05'],['Yaw (rad)','3.10 ~ 3.1415','3.10 ~ 3.1415'],['레일 이동 (mm)','−21.5 ~ +23.4','−60 ~ +55']]
t=a.table(cellText=rows,cellLoc='left',colWidths=[.35,.325,.325],bbox=[0,.24,1,.61]); t.auto_set_font_size(False); t.set_fontsize(10.5)
for (r,c),cell in t.get_celld().items():
    cell.set_edgecolor('#d6dbe3'); cell.set_facecolor('#e6ebf2' if r==0 else '#f5f5f5'); cell.set_text_props(color=ink)
lines(a,['보드 Z=1.14 m, Roll/Pitch=0 고정','레일 local yaw=0 · world 방향은 보드를 따름'],y=.16,step=.10)

a=panel(3,'04  로봇 · 케이블 초기 상태','기준 자세에 잡음 추가 · Trial마다 재생성')
lines(a,['로봇 6개 관절: 각각 ±4° 균등 잡음','그리퍼 내 케이블 위치: XYZ 각각 ±2 mm','케이블 회전: 임의 축, 최대 0.04 rad (2.29°)'],step=.16)
a.text(0,.22,'회전 잡음의 선택 규칙',transform=a.transAxes,weight='bold',color=blue,fontsize=13)
a.text(0,.07,'축: 정규 난수 벡터 정규화\n각도: 0.04 × U^(1/3),  U ~ Uniform(0, 1)\n기준 자세에 local 회전으로 합성',transform=a.transAxes,color=ink,fontsize=12)

a=panel(4,'05  촬영 위치 · 각도','기본 near-port 정책 · 포트 local 좌표계')
lines(a,['XYZ/RPY를 구간 분할 후 무작위 샘플링','위치 tier: 50 / 10 / 5 / 2 mm','기본 40개 계획 → tier별 10개 배정'],step=.14)
lines(a,['최대 tier: ΔX, ΔY = ±50 mm; ΔZ = 0–50 mm','Roll/Pitch = ±25°; Yaw = ±35°','작은 tier는 위치·각도 범위를 비례 축소'],y=.34,step=.10)
a.text(0,-.015,'기준 offset 0 샘플 1개 포함 · 기본 Z 기준 1 mm\n계획 수 ≠ 저장 수: 가시성·도달·안전 조건 적용',transform=a.transAxes,color=blue,fontsize=10.5)

a=panel(5,'06  조명 · 배경','범위 밖 재추출: 절단 정규 분포 (±3σ)')
b=a.inset_axes([.04,.62,.92,.24]); x=[-3+i*.03 for i in range(201)]; y=[math.exp(-v*v/2)/math.sqrt(2*math.pi)/math.erf(3/math.sqrt(2)) for v in x]
b.plot(x,y,color=orange,lw=2); b.fill_between(x,y,color=orange,alpha=.15); b.set(xticks=[-3,0,3],xticklabels=['−3σ','μ','+3σ'],yticks=[]); b.spines[['top','right','left']].set_visible(False); b.set_facecolor('#f5f5f5')
lines(a,['Ambient: 0–0.08  ·  Background: 0.08–0.20','광원 강도: 기준값 × 0.65–1.35','광원 이동: X/Y ±0.25 m, Z ±0.20 m','RGB: 각 채널 1 + 잡음(±0.12) → [0,1] 제한'],y=.46,step=.115)
a.text(0,-.015,'μ = 범위 중간값, σ = 범위 폭 / 6\n최종 RGB 범위 0.88–1.00 · 광원 Z ≥ 0.5 m',transform=a.transAxes,color=blue,fontsize=10.5)

fig.text(.045,.058,'기준: phy_data_collection 기본 설정 · min_card_count=1 · near-port · 조명 랜덤화 ON',fontsize=11,color=muted)
fig.text(.045,.033,'이론적 생성 규칙이며 실측 빈도 그래프가 아님. CLI 설정에 따라 범위 변경 가능.  |  Sources: runner/{constants,scenario,world,cli}.py · policy/motion.py',fontsize=10,color=muted)
fig.savefig(OUT/'randomization-overview.png',dpi=180,facecolor=fig.get_facecolor())
fig.savefig(OUT/'randomization-overview.svg',facecolor=fig.get_facecolor())
svg=(OUT/'randomization-overview.svg').read_text(); svg=svg[svg.index('<svg'):]
svg=svg.replace('<svg ', '<svg role="img" aria-labelledby="randomization-title randomization-desc" ',1)
start=svg.index('>')+1
svg=svg[:start]+'<title id="randomization-title">Randomizable Dataset Generator</title><desc id="randomization-desc">현재 기본 설정의 카드 조합, 목표 포트, 보드와 레일 위치, 로봇과 케이블, 촬영 자세, 조명 랜덤화 범위. 카드 확률은 실측 빈도가 아닌 이론값.</desc>'+svg[start:]
(OUT/'randomization-overview.svg').write_text(svg)
(OUT/'randomization-overview.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Randomizable Dataset Generator</title><style>body{margin:0;background:#f5f5f5}svg{width:100%;height:auto;display:block}</style>'+svg+'</html>')
print(OUT/'randomization-overview.png')
