# 조명 랜덤화의 공통 가우시안 분포만 포트폴리오용 그래프로 생성한다.
# X축은 변수별 평균 기준 편차, Y축은 확률 밀도다.
# 실제 적용 범위는 강도·Ambient·Background·광원 위치를 작은 주석으로 표시한다.
# 출력은 같은 폴더의 PNG, SVG, HTML이며 별도 실행 인자는 없다.
from pathlib import Path
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT = Path(__file__).resolve().parent
font = font_manager.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
plt.rcParams.update({"font.family": font.get_name(), "svg.fonttype": "path", "axes.unicode_minus": False})
paper, ink, muted, orange = "#ffffff", "#2d3142", "#4f5d75", "#eb6c36"
fig, ax = plt.subplots(figsize=(11, 6.2), facecolor=paper)
fig.subplots_adjust(left=.14, right=.96, bottom=.25, top=.82)
x = [-3 + i * .01 for i in range(601)]
y = [math.exp(-value * value / 2) / math.sqrt(2 * math.pi) for value in x]
ax.plot(x, y, color=orange, linewidth=3)
ax.fill_between(x, y, color=orange, alpha=.16)
ax.set_facecolor(paper)
ax.set_xlim(-3.15, 3.15)
ax.set_ylim(0, .45)
ax.set_xticks([-3, 0, 3], ["−3σ", "μ", "+3σ"])
ax.set_yticks([0, .15, .30, .45], ["0", ".15", ".30", ".45"])
ax.set_xlabel("조명 랜덤화 값 (변수별 μ 기준 편차)", fontsize=12, labelpad=12, color=ink)
ax.set_ylabel("확률 밀도", fontsize=12, labelpad=12, color=ink)
ax.grid(axis="y", color=ink, alpha=.10, linewidth=.8)
ax.spines[["top", "right"]].set_visible(False)
ax.spines[["left", "bottom"]].set_color(ink)
ax.tick_params(colors=muted, labelsize=10)
fig.text(.14, .92, "조명 랜덤화 · 공통 가우시안 분포", fontsize=22, weight="bold", color=ink)
fig.text(.14, .865, "광원 강도 · Ambient · Background · RGB · 광원 위치에 동일한 방식 적용", fontsize=11, color=muted)
fig.text(.14, .08, "실제 범위: 강도 ×0.65–1.35  ·  Ambient 0–0.08  ·  Background 0.08–0.20  ·  광원 X/Y ±0.25 m, Z ±0.20 m", fontsize=9.5, color=muted)
fig.text(.14, .045, "범위 밖 값은 재추출되며, 곡선은 변수별 공통 분포 형태를 나타냄", fontsize=9, color=muted)
stem = OUT / "lighting-gaussian"
fig.savefig(stem.with_suffix(".png"), dpi=220, facecolor=paper)
fig.savefig(stem.with_suffix(".svg"), facecolor=paper)
svg = stem.with_suffix(".svg").read_text()
svg = svg[svg.index("<svg"):]
svg = svg.replace("<svg ", '<svg role="img" aria-labelledby="lighting-title lighting-desc" ', 1)
pos = svg.index(">") + 1
svg = svg[:pos] + '<title id="lighting-title">조명 랜덤화 공통 가우시안 분포</title><desc id="lighting-desc">조명 랜덤화 값의 평균 기준 편차와 확률 밀도. 강도, Ambient, Background, RGB, 광원 위치에 적용.</desc>' + svg[pos:]
stem.with_suffix(".svg").write_text(svg)
stem.with_suffix(".html").write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><title>조명 랜덤화 가우시안</title><style>body{margin:0;background:#f5f5f5}svg{display:block;width:100%;height:auto}</style>' + svg + "</html>")
print(stem.with_suffix(".png"))
