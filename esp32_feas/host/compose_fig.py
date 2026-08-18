#!/usr/bin/env python3
"""Compose the 4-panel embedded feasibility figure for the paper."""
import os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import glob

HERE = os.path.dirname(os.path.abspath(__file__))
EMB = os.path.abspath(os.path.join(HERE, "..", "results", "embedded"))
OUT = os.path.abspath(os.path.join(HERE, "..", "results", "figures", "fig_emb.png"))
os.makedirs(os.path.dirname(OUT), exist_ok=True)

# import the analyzer's loaders
sys.path.insert(0, HERE)
from analyze import load_B, load_H, load_C

B, BF, float_avg, fixed_avg = load_B()
ledc, hn, htr, hadcv, hadcr = load_H()
C, series = load_C()

# aggregate: per-Ts means for compute
x = np.arange(3)
fa = float_avg.mean(axis=1)
fx = fixed_avg.mean(axis=1)

fig, axs = plt.subplots(2, 2, figsize=(6.8, 4.6))
a = axs[0, 0]
w = 0.35
a.bar(x - w / 2, fa / 1000, w, label="float32", color="#7570b3")
a.bar(x + w / 2, fx / 1000, w, label="int32 Q", color="#1b9e77")
for xi, v in zip(x - w / 2, fa / 1000): a.text(xi, v + 0.04, f"{v:.2f}", ha="center", fontsize=7)
for xi, v in zip(x + w / 2, fx / 1000): a.text(xi, v + 0.04, f"{v:.2f}", ha="center", fontsize=7)
a.set_xticks(x); a.set_xticklabels(["10", "20", "50"])
a.set_xlabel("$T_s$ ($\\mu$s)"); a.set_ylabel("pipeline ($\\mu$s)")
a.legend(fontsize=7); a.set_title("(a) Controller compute", fontsize=9)

a = axs[0, 1]
bits = [b[0] for b in ledc]; hz = [b[1] for b in ledc]
a.scatter(bits, np.array(hz) / 1e3, s=25, color="#d95f02")
a.axhline(100, color="gray", ls="--", lw=1)
a.text(5.2, 101.5, "requested 100 kHz", fontsize=7, color="gray")
a.set_xlabel("LEDC resolution (bit)"); a.set_ylabel("fsw (kHz)")
a.set_ylim(0, 110); a.set_xticks(range(1, 13))
a.set_title("(b) DPWM ceiling @100 kHz", fontsize=9)
a.annotate("8-bit max", xy=(8, 100), xytext=(9.5, 60), fontsize=8,
           arrowprops=dict(arrowstyle="->", lw=0.8, color="black"))

a = axs[1, 0]
Ns = [n[0] for n in hn]; sg = [n[2] for n in hn]
a.semilogx(Ns, sg, "o-", color="#1b9e77")
a.set_xlabel("oversampling N per step"); a.set_ylabel(r"$\sigma$ of mean (V)")
a.grid(True, which="both", alpha=0.3)
a.text(2, 1.15, r"$\sigma \approx 1.65/\sqrt{N}$", fontsize=8)
a.set_title("(c) ADC acquisition noise", fontsize=9)

a = axs[1, 1]
m = 1
for retune, lab, col in ((1, "retuned", "#1b9e77"), (0, "frozen gains", "#d95f02")):
    key = None
    for k in series:
        if k[0] == m and k[1] == 0 and k[3] == retune:
            key = k; break
    if key is None: continue
    s = np.array(series[key])
    a.plot(s[:, 0] * 1e-3, s[:, 2], lw=1.0, label=lab, color=col)
a.axhline(0.5, color="gray", ls=":", lw=0.8)
a.text(0.05, 0.505, "d=0.5", fontsize=7, color="gray")
a.set_xlabel("time (s)"); a.set_ylabel("duty d")
a.set_ylim(0, 1.05)
a.legend(fontsize=7, loc="upper right")
a.set_title("(d) Closed loop (real PWM→ADC)", fontsize=9)

fig.tight_layout()
fig.savefig(OUT, dpi=200)
print("wrote", OUT)