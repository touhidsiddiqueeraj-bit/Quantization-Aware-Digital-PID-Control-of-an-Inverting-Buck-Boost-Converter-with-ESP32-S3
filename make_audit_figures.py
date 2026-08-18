"""Regenerate figures affected by the audit runs.

- fig2_sampling.png  -> sampling-period sweep extended across the critical
  boundary (Ts/Tsw = 1..4 at 16-bit ADC/PWM). Include IAE panel (audit 7).
- fig8_perf_map.png  -> performance classification map (audit 10) with the
  explicit thresholds used in the paper: Excellent / Acceptable / Marginal / Unusable.
"""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

OUT = "results"
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                     "figure.dpi": 140})
rows = list(csv.DictReader(open(f"{OUT}/results.csv")))
m0 = next(r for r in rows if r["kind"] == "ideal_digital")


# ---- fig2: critical sampling boundary (16-bit ADC/PWM) ----
sel = [r for r in rows
       if (r["kind"].startswith("sampling") or r["kind"].startswith("boundary"))
       and r["adc"] == "16" and r["pwm"] == "16"]
seen = {}
for r in sel:
    seen.setdefault(float(r["ts"]), r)
sel = [seen[k] for k in sorted(seen)]
xs = [float(r["ts"]) for r in sel]
settle = [float(r["t_settle_ms"]) for r in sel]
sse = [float(r["ss_error_mV"]) for r in sel]
lc = [float(r["limit_cycle_mV"]) for r in sel]
iae = [float(r["iae"]) for r in sel]

fig, axs = plt.subplots(2, 2, figsize=(6.2, 4.6), sharex=True)
def limline(ax, y, label, ylab):
    ax.plot(xs, y, "o-", lw=1.2, ms=4, color="tab:blue")
    ax.set_ylabel(ylab)
    # x axhline only where scale allows
selx = np.array(xs)
axs[0, 0].plot(xs, settle, "o-", lw=1.2, ms=4, color="tab:blue")
axs[0, 0].axvline(3.5, color="r", ls="--", lw=0.8)
axs[0, 0].set_ylabel("2% settling [ms]")
axs[0, 1].semilogy(xs, [abs(v) for v in sse], "o-", lw=1.2, ms=4, color="tab:blue")
axs[0, 1].set_ylabel("|ss error| [mV]")
axs[0, 1].set_ylim(1e-1, 2e4)
axs[1, 0].semilogy(xs, lc, "o-", lw=1.2, ms=4, color="tab:blue")
axs[1, 0].set_ylabel("limit cycle [mV]")
axs[1, 0].set_xlabel("Ts/Tsw  —  (×10 µs)")
axs[1, 1].semilogy(xs, iae, "o-", lw=1.2, ms=4, color="tab:blue")
axs[1, 1].set_ylabel("IAE [V·s]")
axs[1, 1].set_xlabel("Ts/Tsw  —  (×10 µs)")
axs[0, 0].set_xticks([1, 2, 3, 3.5, 4]); axs[1, 0].set_xticks([1, 2, 3, 3.5, 4])
axs[1, 1].set_xticks([1, 2, 3, 3.5, 4])
fig.suptitle("Critical sampling boundary — ideal ADC/PWM (Ts/Tsw ≥ 4× loses regulation)")
fig.tight_layout()
fig.savefig(f"{OUT}/figures/fig2_sampling.png")
plt.close(fig)

# ---- fig8: performance classification map (audit 10) ----
def classify(r):
    ts = float(r["t_settle_ms"])
    ss = abs(float(r["ss_error_mV"]))
    lc = float(r["limit_cycle_mV"])
    ov = float(r["overshoot_pct"])
    if ts >= 50.0 or ss > 1000.0:      # never settles in the 60 ms window
        return 3                        # Unusable
    if ts <= 4.0 and ss <= 60.0 and lc <= 60.0 and ov <= 5.0:
        return 0                        # Excellent
    if ts <= 6.0 and ss <= 120.0 and lc <= 120.0:
        return 1                        # Acceptable
    return 2                            # Marginal

cmap = ListedColormap(["#2e7d32", "#a68b00", "#ef6c00", "#b71c1c"])
comb = [r for r in rows if r["kind"] == "combined"]
fig, axs = plt.subplots(1, 3, figsize=(9.5, 2.9))
for i, ts in enumerate((1, 2, 5)):
    sub = [r for r in comb if r["ts"] == str(ts)]
    M = np.zeros((4, 4))
    for r in sub:
        M[6 - int(r["pwm"]) // 2, int(r["adc"]) // 2 - 3] = classify(r)
    im = axs[i].imshow(M, cmap=cmap, vmin=0, vmax=3, aspect="auto")
    axs[i].set_title(f"Ts = {ts}×Tsw")
    axs[i].set_xticks(range(4)); axs[i].set_xticklabels([12, 10, 8, 6])
    axs[i].set_yticks(range(4)); axs[i].set_yticklabels([6, 8, 10, 12])
    axs[i].set_xlabel("ADC bits"); axs[i].set_ylabel("PWM bits")
fig.suptitle("Performance map — green: excellent · yellow: acceptable · orange: marginal · red: unusable")
fig.tight_layout()
fig.savefig(f"{OUT}/figures/fig8_perf_map.png")
plt.close(fig)
print("figures regenerated")