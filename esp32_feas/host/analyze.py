#!/usr/bin/env python3
"""Analyze ESP32-S3 feasibility captures -> stats CSVs + paper figures.

Reads results/embedded/{B,C,H}.csv, writes:
  results/embedded/stats.csv        key headline numbers
  results/embedded/fig_ledc.png     LEDC resolvable frequency vs bits @100kHz
  results/embedded/fig_adc_noise.png ADC noise floor vs oversampling N
  results/embedded/fig_closed.png   closed-loop time series (frozen vs retuned)
  results/embedded/fig_latency.png  compute pipeline latency (float vs fixed)
"""
import csv, io, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(__file__), "..", "results", "embedded")
OUT = os.path.abspath(OUT)

def read_csv(fn):
    rows = []
    with open(os.path.join(OUT, fn)) as f:
        for line in f:
            line = line.strip()
            if not line.startswith("CSV,"): continue
            rows.append(line.split(","))
    return rows

# ---------------- mode B: compute latency ----------------
def load_B():
    rows = read_csv("B.csv")
    B = [r for r in rows if r[1] == "B"]
    BF = [r for r in rows if r[1] == "BF"]
    # columns: B,t,ts_us,adc,pwm,min,avg,max,p99    BF adds maxdev
    def agg(rs, idx):
        return np.array([[float(r[idx]) for r in rs if float(r[2]) == t] for t in (0, 1, 2)], dtype=object)
    float_avg = np.array([[float(r[6]) for r in B if float(r[2]) == t] for t in (0, 1, 2)])
    fixed_avg = np.array([[float(r[6]) for r in BF if float(r[2]) == t] for t in (0, 1, 2)])
    return B, BF, float_avg, fixed_avg

# ---------------- mode H ----------------
def load_H():
    rows = read_csv("H.csv")
    ledc = [(int(r[2]), int(r[3]), int(r[4])) for r in rows if r[1] == "H"]
    hn = [(int(r[2]), float(r[3]), float(r[4])) for r in rows if r[1] == "HN"]
    htr = [(int(r[2]), float(r[3]), float(r[4])) for r in rows if r[1] == "HTR"]
    hadcv = [r for r in rows if r[1] == "HADC"]
    hadcr = [r for r in rows if r[1] == "HADCRAW"]
    return ledc, hn, htr, hadcv, hadcr

# ---------------- mode C ----------------
def load_C():
    rows = read_csv("C.csv")
    summ = [r for r in rows if r[1] == "C"]
    tsr = [r for r in rows if r[1] == "CT"]
    # summary cols: C,mult,pass,ts_nom,ss_err,ripple_mv,settle_ms,d_mean,d_ripple,rate_khz,
    #               loop_min,loop_avg,loop_max,p99_ns,avg_n,ts_meas_us,retune
    parsed = []
    for r in summ:
        parsed.append(dict(mult=int(r[2]), pass_=int(r[3]), ts_nom=float(r[4]), ss_err=float(r[5]),
                           ripple_mv=float(r[6]), settle_ms=float(r[7]), d_mean=float(r[8]),
                           d_ripple=float(r[9]), rate_khz=float(r[10]), loop_avg=int(float(r[12])),
                           avg_n=int(r[15]), ts_meas_us=float(r[16]), retune=int(r[17])))
    # time series: header CT,mult,pass,n,avg_n,retune (7)
#             rows   CT,mult,pass,i,avg_n,retune,t,meas,duty (10)
    series = {}
    cur = None
    for r in tsr:
        if len(r) == 7:  # header
            cur = (int(r[2]), int(r[3]), int(r[5]), int(r[6]))  # mult,pass,avg_n,retune
            series[cur] = []
        elif len(r) == 10 and cur:
            series[cur].append((float(r[7]), float(r[8]), float(r[9])))  # t,meas,duty
    return parsed, series

def main():
    B, BF, float_avg, fixed_avg = load_B()
    ledc, hn, htr, hadcv, hadcr = load_H()
    C, series = load_C()

    # statistical summary
    stats = []
    def S(k, v): stats.append((k, v))

    # compute: averaged across (adc_bits,pwm_bits) per Ts regime
    S("float_avg_ns", np.mean(float_avg)); S("float_avg_ns_min", float_avg.min())
    S("float_avg_ns_max", float_avg.max())
    S("fixed_avg_ns", np.mean(fixed_avg)); S("fixed_avg_ns_min", fixed_avg.min())
    S("fixed_avg_ns_max", fixed_avg.max())
    # across only the 12/12 combo (paper's nominal precision)
    B1212 = [r for r in B if r[4] == "12" and r[5] == "12"]
    S("float_12_avg_ns", np.mean([float(r[6]) for r in B1212]))
    BF1212 = [r for r in BF if r[4] == "12" and r[5] == "12"]
    S("fixed_12_avg_ns", np.mean([float(r[6]) for r in BF1212]))
    S("fixed_max_dev_duty", max(float(r[10]) for r in BF))

    # LEDC ceiling
    ok = [b for b in ledc if b[2] == 1 and b[1] != 0]
    S("ledc_max_bits_100k", max(b[0] for b in ok))
    # ADC latency
    v0 = hadcv[0]; r0 = hadcr[0]
    S("adc_mv_avg_us", float(v0[3])); S("adc_mv_max_us", float(v0[4]))
    S("adc_raw_avg_us", float(r0[3])); S("adc_raw_max_us", float(r0[4]))
    # noise floor
    n1 = next(n for n in hn if n[0] == 1); n128 = next(n for n in hn if n[0] == 128)
    S("noise_N1_sigma_V", n1[2]); S("noise_N128_sigma_V", n128[2])
    S("adc_mean_V_at_N128", n128[1])

    # closed loop: retuned, all Ts
    cr = [c for c in C if c["retune"] == 1]
    S("loop_rate_khz", np.mean([c["rate_khz"] for c in cr]))
    S("loop_ts_meas_us", np.mean([c["ts_meas_us"] for c in cr]))
    S("loop_ss_err_V_mean", np.mean([abs(c["ss_err"]) for c in cr]))
    S("loop_d_mean", np.mean([c["d_mean"] for c in cr]))
    S("loop_d_ripple", np.mean([c["d_ripple"] for c in cr]))
    cf = [c for c in C if c["retune"] == 0]
    S("loop_ss_err_frozen_mean", np.mean([abs(c["ss_err"]) for c in cf]))
    S("loop_d_ripple_frozen", np.mean([c["d_ripple"] for c in cf]))

    print("[[embedded stats]]")
    for k, v in stats:
        print(f"{k}={v:.5g}" if isinstance(v, float) else f"{k}={v}")

    # ---------------- figures ----------------
    # fig_ledc: achievable freq vs requested resolution
    fig, ax = plt.subplots(figsize=(4, 2.6))
    bits = [b[0] for b in ledc]; hz = [b[1] for b in ledc]
    ax.scatter(bits, np.array(hz) / 1e3, s=40, color="#d95f02")
    ax.axhline(100, color="gray", ls="--", lw=1)
    ax.text(10.5, 101, "requested 100 kHz", fontsize=8, ha="right", color="gray")
    ax.set_xlabel("LEDC resolution (bit)"); ax.set_ylabel("achieved fsw (kHz)")
    ax.set_ylim(0, 110); ax.set_xticks(range(1, 13))
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_ledc.png"), dpi=200); plt.close(fig)

    # fig_adc_noise: sigma vs N (log-x)
    fig, ax = plt.subplots(figsize=(4, 2.6))
    Ns = [n[0] for n in hn]; sg = [n[2] for n in hn]
    ax.semilogx(Ns, sg, "o-", color="#1b9e77")
    ax.set_xlabel("oversampling N per step"); ax.set_ylabel(r"$\sigma$ of mean (V)")
    ax.grid(True, which="both", alpha=0.3)
    ax.text(2, 1.25, "σ ≈ 1.65/√N", fontsize=9)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_adc_noise.png"), dpi=200); plt.close(fig)

    # fig_latency: float vs fixed avg per Ts regime
    fig, ax = plt.subplots(figsize=(4, 2.6))
    x = np.arange(3); labels = ["10", "20", "50"]
    fa = float_avg.mean(axis=1); fx = fixed_avg.mean(axis=1)
    w = 0.35
    ax.bar(x - w / 2, fa, w, label="float32", color="#7570b3")
    ax.bar(x + w / 2, fx, w, label="int32 fixed", color="#1b9e77")
    for xi, v in zip(x - w / 2, fa): ax.text(xi, v + 300, f"{v/1000:.2f}µs", ha="center", fontsize=8)
    for xi, v in zip(x + w / 2, fx): ax.text(xi, v + 300, f"{v/1000:.2f}µs", ha="center", fontsize=8)
    ax.axhline(10000, color="gray", ls="--", lw=1); ax.text(2.2, 10200, "Ts=10µs", fontsize=8, color="gray")
    ax.set_xticks(x); ax.set_xticklabels(labels); ax.set_yscale("log")
    ax.set_xlabel("control period Ts (µs)"); ax.set_ylabel("pipeline latency (ns)")
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_latency.png"), dpi=200); plt.close(fig)

    # fig_closed: retuned vs frozen, float pass, any available mult (default 1), time series
    fig, axs = plt.subplots(2, 1, figsize=(6.5, 4.5), sharex=True)
    mult = 1
    has = False
    for retune, lab, col in ((1, "retuned (slow integral)", "#1b9e77"), (0, "frozen paper gains", "#d95f02")):
        # find a float-pass (pass_=0) series for this mult and any avg_n in capture
        key = None
        for k in series:
            if k[0] == mult and k[1] == 0 and k[3] == retune:
                key = k; break
        if key is None: continue
        s = np.array(series[key])
        axs[0].plot(s[:, 0] * 1e-3, s[:, 1], lw=1.2, label=lab, color=col)  # ms -> s
        axs[1].plot(s[:, 0] * 1e-3, s[:, 2], lw=1.2, color=col)
        has = True
    axs[0].axhline(-12, color="gray", ls=":"); axs[0].text(0.05, -11.6, "VREF=−12 V", fontsize=8, color="gray")
    axs[0].set_ylabel("sensed $V_{out}$ (V)")
    axs[1].set_ylabel("duty d"); axs[1].set_xlabel("time (s)")
    if has: axs[0].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig_closed.png"), dpi=200); plt.close(fig)

    with open(os.path.join(OUT, "stats.csv"), "w") as f:
        w = csv.writer(f); w.writerow(["key", "value"])
        for k, v in stats: w.writerow([k, v])
    print(f"\nwrote {os.path.join(OUT, 'stats.csv')}")

if __name__ == "__main__":
    main()