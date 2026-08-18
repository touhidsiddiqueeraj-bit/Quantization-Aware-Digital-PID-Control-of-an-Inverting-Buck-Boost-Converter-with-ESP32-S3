#!/usr/bin/env python3
"""Analyze firmware v2 captures (modes L/R/S/X/A) -> stats + journal figures.

Reads results/embedded/{L,R,S,X,A}.log, writes:
  results/embedded/v2_stats.csv            headline numbers
  results/embedded/fig_v2_latency.png      latency breakdown bars (T_adc/T_comp/T_pwm)
  results/embedded/fig_v2_effres.png       IAE vs effective ADC/PWM resolution
  results/embedded/fig_v2_sched.png        settle/IAE vs achievable schedule (frozen vs retuned)
  results/embedded/fig_v2_latinj.png       IAE/settle vs injected latency
  results/embedded/fig_v2_adaptive.png     adaptive mode timeline + error + SR
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "results", "embedded"))

def read_csv(fn):
    rows = []
    with open(os.path.join(OUT, fn)) as f:
        for line in f:
            line = line.strip()
            if not line.startswith("CSV,"):
                continue
            rows.append(line.split(","))
    return rows

# ---------------- mode L ----------------
def load_L():
    rows = read_csv("L.log")
    out = {}
    for r in rows:
        if r[1] == "L":
            out[r[2]] = dict(min_us=float(r[3]), avg_us=float(r[4]), max_us=float(r[5]))
    return out      # keys: adc_mv, adc_raw, comp_f32, comp_i32, pwm

# ---------------- mode R ----------------
def load_R():
    rows = read_csv("R.log")
    out = []
    for r in rows:
        if r[1] == "R":
            out.append(dict(adc=int(r[2]), pwm=int(r[3]),
                            ts_us=float(r[4]), ss_err_mv=float(r[5]),
                            ripple_mv=float(r[6]), settle_ms=float(r[7]),
                            d_mean=float(r[8]), d_ripple=float(r[9]), iae=float(r[10])))
    return out

# ---------------- mode S ----------------
def load_S():
    rows = read_csv("S.log")
    out = []
    for r in rows:
        if r[1] == "S":
            out.append(dict(avg=int(r[2]), retune=int(r[3]), ts_us=float(r[4]),
                            rate_khz=float(r[5]), settle_ms=float(r[6]),
                            ss_err=float(r[7]), ripple_mv=float(r[8]),
                            d_ripple=float(r[9]), d_mean=float(r[10]), iae=float(r[11])))
    return out

# ---------------- mode X ----------------
def load_X():
    rows = read_csv("X.log")
    out = []
    for r in rows:
        if r[1] == "X":
            out.append(dict(dly_us=float(r[2]), ts_us=float(r[3]), settle_ms=float(r[4]),
                            ss_err=float(r[5]), ripple_mv=float(r[6]), iae=float(r[7])))
    return out

# ---------------- mode A ----------------
def load_A():
    rows = read_csv("A.log")
    out = []
    for r in rows:
        if r[1] == "A":
            out.append(dict(t_ms=float(r[2]), v=float(r[3]), d=float(r[4]), mode=int(r[5])))
    return out

def main():
    L = load_L()
    R = load_R()
    S = load_S()
    X = load_X()
    A = load_A()
    print(f"L={list(L)} R={len(R)} S={len(S)} X={len(X)} A={len(A)}")
    assert L and R and S and X and A

    stats = []
    def S_(k, v): stats.append((k, v))

    # latency
    S_("lat_adc_raw_avg_us", L["adc_raw"]["avg_us"])
    S_("lat_adc_mv_avg_us", L["adc_mv"]["avg_us"])
    S_("lat_comp_f32_avg_us", L["comp_f32"]["avg_us"])
    S_("lat_comp_i32_avg_us", L["comp_i32"]["avg_us"])
    S_("lat_pwm_avg_us", L["pwm"]["avg_us"])
    lat_total = L["adc_raw"]["avg_us"] + L["comp_i32"]["avg_us"] + L["pwm"]["avg_us"]
    S_("lat_loop_est_us", lat_total)
    S_("cpu_pct_f32", L["comp_f32"]["avg_us"] / lat_total * 100.0)
    S_("cpu_pct_i32", L["comp_i32"]["avg_us"] / lat_total * 100.0)

    # effective resolution: IAE across cells
    rmin = min(r["iae"] for r in R); rmax = max(r["iae"] for r in R)
    S_("effres_iae_min", rmin); S_("effres_iae_max", rmax)
    S_("effres_sserr_max_mv", max(abs(r["ss_err_mv"]) for r in R))

    # schedule sweep: frozen vs retuned
    s_frozen = [r for r in S if r["retune"] == 0]
    s_retuned = [r for r in S if r["retune"] == 1]
    S_("sched_frozen_iae_max", max(r["iae"] for r in s_frozen))
    S_("sched_retuned_iae_max", max(r["iae"] for r in s_retuned))
    S_("sched_retuned_sserr_max_V", max(abs(r["ss_err"]) for r in s_retuned))

    # latency injection: IAE growth
    x0 = next(r for r in X if r["dly_us"] == 0)
    x1 = next(r for r in X if r["dly_us"] == max(r["dly_us"] for r in X))
    S_("lat_inj_iae_0", x0["iae"]); S_("lat_inj_iae_max", x1["iae"])
    S_("lat_inj_iae_mult", x1["iae"] / x0["iae"] if x0["iae"] else 0)

    # adaptive: mode residency + SR
    counts = {0: 0, 1: 0, 2: 0}
    durs = {0: 0.0, 1: 0.0, 2: 0.0}
    for i in range(len(A)):
        counts[A[i]["mode"]] += 1
        if i > 0:
            durs[A[i - 1]["mode"]] += A[i]["t_ms"] - A[i - 1]["t_ms"]
    tend = A[-1]["t_ms"]
    S_("adaptive_n", len(A))
    S_("adaptive_t_ms", tend)
    for m, name in ((0, "fast"), (1, "slow"), (2, "dither")):
        S_(f"adaptive_{name}_pct", durs[m] / tend * 100.0)
    f_avg = len(A) / (tend / 1e3)
    f_fast = 1 / 0.864e-3
    S_("adaptive_f_avg_hz", f_avg)
    S_("adaptive_f_fast_hz", f_fast)
    S_("adaptive_SR", 1.0 - f_avg / f_fast)

    print("[[v2 stats]]")
    for k, v in stats:
        print(f"{k}={v:.5g}" if isinstance(v, float) else f"{k}={v}")

    # ---------------- figures ----------------
    # fig_v2_latency: breakdown bars
    fig, ax = plt.subplots(figsize=(4.6, 2.6))
    keys = ["adc_raw", "adc_mv", "comp_i32", "comp_f32", "pwm"]
    labs = ["ADC raw", "ADC mV", "PID int32", "PID float", "PWM write"]
    vals = [L[k]["avg_us"] for k in keys]
    colors = ["#49a9c8", "#8ec4d4", "#1b9e77", "#7570b3", "#d95f02"]
    ax.bar(range(len(vals)), vals, color=colors)
    for i, v in enumerate(vals):
        ax.text(i, v + 3, f"{v:.1f}", ha="center", fontsize=8)
    ax.set_xticks(range(len(vals))); ax.set_xticklabels(labs, fontsize=8)
    ax.set_ylabel("avg latency (µs)"); ax.set_yscale("log")
    ax.set_title("Measured control-loop latency breakdown (ESP32-S3, 240 MHz)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_v2_latency.png"), dpi=200); plt.close(fig)

    # fig_v2_effres: IAE and ss_err vs ADC bits (fixed PWM 8)
    fig, ax = plt.subplots(1, 2, figsize=(8, 2.8))
    adc8 = [r for r in R if r["pwm"] == 8]
    x = [r["adc"] for r in adc8]
    ax[0].plot(x, [r["iae"] for r in adc8], "o-", color="#1b9e77")
    ax[0].set_xlabel("effective ADC resolution (bit)"); ax[0].set_ylabel("IAE")
    ax[0].invert_xaxis()
    ax[0].scatter(x, [r["iae"] for r in adc8], s=40)
    pw8 = [r for r in R if r["adc"] == 8]
    xp = [r["pwm"] for r in pw8]
    ax[1].plot(xp, [r["iae"] for r in pw8], "s-", color="#d95f02")
    ax[1].set_xlabel("effective PWM resolution (bit)"); ax[1].set_ylabel("IAE")
    ax[1].invert_xaxis()
    fig.suptitle("Effective-resolution sweep at N=64 schedule (noise floor $\\sigma\\approx0.95$ V)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_v2_effres.png"), dpi=200); plt.close(fig)

    # fig_v2_sched: settle/IAE vs schedule (frozen vs retuned)
    fig, ax = plt.subplots(1, 2, figsize=(8, 2.8))
    xs = [r["avg"] for r in s_frozen]
    for rs, col, mk, lab in ((s_frozen, "#d95f02", "o", "frozen gains"),
                             (s_retuned, "#1b9e77", "s", "retuned")):
        xr = [r["avg"] for r in rs]
        ax[0].plot(xr, [min(r["settle_ms"], 9000) for r in rs], mk + "-", color=col, label=lab)
        ax[1].plot(xr, [r["iae"] for r in rs], mk + "-", color=col, label=lab)
    for a in ax:
        a.set_xlabel("oversampling N per step (log)")
        a.set_xscale("log"); a.legend(fontsize=8)
    ax[0].set_ylabel("settle (ms, cap 9000)"); ax[1].set_ylabel("IAE")
    ax[0].axvspan(1, 8, alpha=0.08, color="tab:blue")
    fig.suptitle("Achievable-schedule sweep (measured ADC-bound periods)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_v2_sched.png"), dpi=200); plt.close(fig)

    # fig_v2_latinj: IAE vs injected latency
    fig, ax = plt.subplots(figsize=(4.2, 2.6))
    xd = [r["dly_us"] for r in X]
    ax.plot(xd, [r["iae"] for r in X], "o-", color="#7570b3")
    ax.set_xlabel("injected compute delay (µs)"); ax.set_ylabel("IAE")
    ax.set_title("Latency injection at raw schedule (retuned, ki×ts const)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_v2_latinj.png"), dpi=200); plt.close(fig)

    # fig_v2_adaptive: error + mode timeline
    fig, ax = plt.subplots(3, 1, figsize=(7, 6), sharex=True)
    t = np.array([r["t_ms"] for r in A]) / 1e3
    e = np.array([r["d"] for r in A])
    v = np.array([r["v"] for r in A])
    m = np.array([r["mode"] for r in A])
    ax[0].plot(t, v, lw=0.8, color="#1b9e77")
    ax[0].set_ylabel("sensed V (V)"); ax[0].grid(alpha=0.3)
    ax[1].plot(t, e, lw=0.8, color="#d95f02")
    ax[1].set_ylabel("duty"); ax[1].grid(alpha=0.3)
    ax[2].plot(t, m, lw=0.8, color="#7570b3")
    ax[2].set_yticks([0, 1, 2]); ax[2].set_yticklabels(["fast", "slow", "dither"])
    ax[2].set_ylabel("mode"); ax[2].set_xlabel("time (s)")
    for a in ax[:2]:
        a.axvline(2.5, color="tab:red", ls="--", lw=1)
    ax[1].text(2.52, 0.05, "set-point step", fontsize=8, color="tab:red")
    fig.suptitle(f"Adaptive schedule+dithering on ESP32-S3 (SR=$1-f_{{avg}}/f_{{fast}}$ = "
                 f"{stats[15][1]:.2f}, slow mode {stats[13][1]:.0f}%)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_v2_adaptive.png"), dpi=200); plt.close(fig)

    # adaptive CSVs for the paper
    with open(os.path.join(OUT, "v2_stats.csv"), "w") as f:
        w = csv.writer(f); w.writerow(["key", "value"])
        for k, v in stats:
            w.writerow([k, v])
    for name, rows in (("R", R), ("S", S), ("X", X)):
        with open(os.path.join(OUT, f"v2_{name}.csv"), "w", newline="") as f:
            keys = list(rows[0].keys())
            w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)
    print(f"\nwrote {os.path.join(OUT, 'v2_stats.csv')} and fig_v2_*.png")

if __name__ == "__main__":
    main()