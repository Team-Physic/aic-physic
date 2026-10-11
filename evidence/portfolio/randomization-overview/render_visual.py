# Gazebo 예시와 기본 랜덤화 규칙을 한 장의 포트폴리오 그림으로 합성한다.
# 사용자 제공 이미지의 장면은 수정하지 않고 전체를 중앙에 배치한다.
# 출력은 PNG, SVG, HTML이며 확률은 실측이 아닌 이론적 생성 확률이다.
# 범위는 기본 near-port 및 min_card_count=1 설정을 기준으로 표시한다.
# 실행 인자 없음. MPLCONFIGDIR은 matplotlib 캐시 위치를 지정한다.
from pathlib import Path
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT = Path(__file__).resolve().parent
SOURCE = Path('/home/swlinux/Downloads/Robotics Vision Engineer_PPT.png')
font = font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'font.size':11,'svg.fonttype':'path','axes.unicode_minus':False})
ink, muted, orange, blue, paper = '#2d3142','#4f5d75','#eb6c36','#2e5aa8','#f5f5f5'
fig=plt.figure(figsize=(18,11),facecolor=paper)
fig.text(.04,.941,'Randomizable Dataset Generator',fontsize=29,weight='bold',color=ink)
fig.text(.04,.898,'같은 로봇 작업, 다양한 카드 배치 · 목표 · 촬영 자세 · 조명',fontsize=18,color=muted)

def text(x,y,s,size=12,color=ink,weight='normal'):
    fig.text(x,y,s,fontsize=size,color=color,weight=weight,va='top',linespacing=1.6)

def heading(x,y,n,title,distribution):
    text(x,y,f'{n}  {title}',17,ink,'bold')
    text(x,y-.038,distribution,11,blue)

heading(.04,.824,'01','카드 구성','장착 조합 균등 추첨')
ax=fig.add_axes([.065,.572,.175,.172],facecolor=paper)
p=[math.comb(5,k)/31*100 for k in range(1,6)]
assert abs(sum(p)-100)<1e-9
ax.bar(range(1,6),p,color='#cbd3df',edgecolor=muted,width=.72)
for k,v in enumerate(p,1): ax.text(k,v+1.4,f'{v:.1f}',ha='center',fontsize=10,color=ink)
ax.set(ylim=(0,40),xticks=range(1,6),yticks=[0,20,40],xlabel='SFP NIC 카드 수 (장)',ylabel='확률 (%)')
ax.spines[['top','right']].set_visible(False)
text(.04,.525,'SFP 1–5장 · 5개 레일의 31조합\nSC 1–2장 · 2개 레일의 3조합',11)
heading(.04,.45,'02','목표 카드 / 포트','생성된 카드 안에서 균등 선택')
text(.04,.38,'활성 카드 중 목표 1장 선택\nSFP 포트 0 / 1 → 각 50%\nSFP / SC 종류는 실행 시 지정',11)

ax=fig.add_axes([.275,.384,.45,.45]); ax.imshow(plt.imread(SOURCE)); ax.axis('off')
fig.text(.5,.845,'GAZEBO · 생성 장면 예시',ha='center',fontsize=12,color=muted,weight='bold')
# 중앙 캡션은 사진만으로 식별할 수 있는 변화만 설명한다.
fig.text(.5,.374,'카드 배치와 로봇 자세가 달라지는 환경',ha='center',fontsize=14,weight='bold',color=ink)
fig.text(.5,.348,'사용자 제공 예시 · 각 장면의 정확한 설정값은 미확인',ha='center',fontsize=10,color=muted)

heading(.76,.824,'03','조명 / 배경','공통 가우시안 분포')
ax=fig.add_axes([.775,.649,.18,.09],facecolor=paper)
x=[-3+i*.03 for i in range(201)]; y=[math.exp(-v*v/2) for v in x]
ax.plot(x,y,color=orange,lw=2.5); ax.fill_between(x,y,color=orange,alpha=.18)
ax.set(xticks=[-3,0,3],xticklabels=['−3σ','μ','+3σ'],yticks=[],xlabel='조명 랜덤화 값 (변수별 μ 기준 편차)',ylabel='확률 밀도')
ax.xaxis.label.set_fontsize(8.5); ax.yaxis.label.set_fontsize(8.5); ax.tick_params(axis='x',labelsize=8)
ax.spines[['top','right','left']].set_visible(False)
text(.76,.599,'적용 변수별 실제 범위\n강도 ×0.65–1.35 · X/Y ±0.25 m · Z ±0.20 m\nAmbient 0–0.08 · Background 0.08–0.20',9.5)
text(.76,.456,'RGB 채널: 잡음 적용 후 제한\n최종 0.88–1.00',11)
text(.76,.389,'RGB · 광원 위치에도 동일한 방식 적용',9,muted)

fig.add_artist(plt.Line2D([.04,.96],[.303,.303],transform=fig.transFigure,color='#cbd3df',lw=1))
heading(.04,.273,'04','보드 / 레일 위치','균등 분포')
text(.04,.197,'보드 X  SFP 0.13–0.17 / SC 0.15–0.19 m\n보드 Y  SFP −0.25–−0.20 / SC −0.05–0.05 m\nYaw 3.10–3.1415 rad\n레일 이동  SFP −21.5–+23.4 / SC −60–+55 mm',10.5)
heading(.37,.273,'05','촬영 위치 / 각도','구간별 무작위 샘플링 · Stratified')
text(.37,.197,'포트 기준 위치 tier  50 / 10 / 5 / 2 mm\n최대 ΔX/Y ±50 mm · ΔZ 0–50 mm\n최대 Roll/Pitch ±25° · Yaw ±35°\n작은 tier는 위치와 각도 범위를 함께 축소',10.5)
heading(.70,.273,'06','로봇 / 케이블','위치·관절 균등 잡음 / 임의 축 회전')
text(.70,.197,'로봇 6관절  각각 ±4°\n그리퍼 내 케이블 XYZ  각각 ±2 mm\n케이블 회전  임의 축 · 최대 2.29°\n기준 로봇·케이블 자세에 잡음 추가',10.5)
text(.04,.048,'기본 설정: min_card_count=1 · near-port · 조명 ON  |  이론적 생성 규칙이며 실제 저장 빈도는 가시성·도달·안전 조건에 따라 달라짐',9,muted)
text(.04,.025,'Source: phy_data_collection / runner/{constants,scenario,world,cli}.py · policy/motion.py',8,muted)
stem=OUT/'randomization-visual'
fig.savefig(stem.with_suffix('.png'),dpi=180,facecolor=paper)
fig.savefig(stem.with_suffix('.svg'),facecolor=paper)
svg=stem.with_suffix('.svg').read_text(); svg=svg[svg.index('<svg'):]
svg=svg.replace('<svg ','<svg role="img" aria-labelledby="rdg-title rdg-desc" ',1)
i=svg.index('>')+1
svg=svg[:i]+'<title id="rdg-title">Randomizable Dataset Generator</title><desc id="rdg-desc">Gazebo 생성 예시와 카드 구성, 목표, 조명, 보드 위치, 촬영 자세, 로봇과 케이블의 기본 랜덤화 범위 및 분포.</desc>'+svg[i:]
stem.with_suffix('.svg').write_text(svg)
stem.with_suffix('.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Randomizable Dataset Generator</title><style>body{margin:0;background:#f5f5f5}svg{display:block;width:100%;height:auto}</style>'+svg+'</html>')
print(stem.with_suffix('.png'))
