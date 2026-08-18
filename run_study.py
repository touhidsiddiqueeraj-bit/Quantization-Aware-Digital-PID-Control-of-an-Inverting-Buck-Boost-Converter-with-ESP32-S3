"""Full study: sanity gate -> tuning -> 79 simulations -> figures -> report data.

Phases (paper pipeline):
  1 Reference:     continuous-ideal baseline, ideal digital reference
  2 Individual:    sampling sweep / ADC sweep / PWM sweep
  3 Interaction:   3x4x4 combined matrix
  4 Robustness:    Vin step + load step on {baseline, ideal digital, worst}
  5 Focused:       critical sampling boundary (Ts/Tsw = 1.5-4 sweep),
                   operating-point robustness (Vin = 10,15 V on 3 configs)
  6 Analysis:      figures, interaction tables, performance map

Note: run_audits.py adds Phases 5 audit runs (boundary sweep + op robustness)
to results.csv after the main 67; run_study.py defines the reproducible core.
"""
import csv
import itertools
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sim_core import (BuckBoost, DigitalPID, ContinuousPID, D_MIN, D_MAX,
                      all_metrics, quantize_abs)

VIN, VREF, L, C, FSW, R = 12.0, -12.0, 100e-6, 220e-6, 100e3, 10.0
T_NOM, T_DIST, T_STEP = 0.06, 0.12, 0.06
OUT = "results"
os.makedirs(OUT + "/figures", exist_ok=True)

plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                     "figure.dpi": 140})


# ---------------- Phase 0: physical sanity gate ----------------

def sanity_gate():
    bb = BuckBoost(vin=VIN, l=L, c=C, fsw=FSW, r=R)
    vc, il = bb.run_open_loop(0.06, 0.5)
    di_l = VIN * 0.5 / (L * FSW)          # analytic CCM iL ripple
    dv_c = il * (1 - 0.5) / (C * FSW)     # analytic vC ripple (approx)
    print("=== Physical sanity gate (open-loop, d = 0.5) ===")
    print(f"  vC_ss = {vc:.3f} V   (ideal -12.000 V,  D/(1-D)*Vin = -12)")
    print(f"  iL_ss = {il:.3f} A   (ideal  2.400 A,  -vC/(R(1-D)) = 2.4)")
    print(f"  analytic ripples: dIL={di_l:.3f} A   dVC~{dv_c:.3f} V")
    assert abs(vc + 12.0) < 0.05 and abs(il - 2.4) < 0.05
    print("  gate PASSED")
    return dict(vc_ss=vc, il_ss=il, dIL_ripple=di_l, dVC_ripple=dv_c)


# ---------------- Phase 1: reproducible PID tuning ----------------

AVG_DT = 1e-6  # average-model step (50k steps per 50 ms)


def run_average_model(pid_k, t_end, vref, n_iter):
    """Averaged (continuous) buck-boost with the same digital controller."""
    kp, ki, kd = pid_k
    pid = DigitalPID(kp, ki, kd, 1.0 / FSW)
    pid.reset(0.5)
    steps_per_sample = max(1, int(round((1.0 / FSW) / AVG_DT)))
    n = int(round(t_end / AVG_DT))
    iL, vC, d = 0.0, 0.0, 0.5
    t, y = np.empty(n // 20), np.empty(n // 20)
    for k in range(n):
        if k % steps_per_sample == 0:
            pid.step(vC, vref)
            d = pid.d
        iL += AVG_DT * (d * VIN + (1 - d) * vC) / L
        vC += AVG_DT * (-(1 - d) * iL - vC / R) / C
        if k % 20 == 0:
            t[k // 20], y[k // 20] = k * AVG_DT, vC
    return t, y


def tune_controller():
    """Deterministic grid search on the average model, then switched-model
    validation. Gains frozen for the entire study."""
    kp_g = [0.005, 0.01, 0.02, 0.05]
    ki_g = [20.0, 100.0, 500.0, 2000.0]
    kd_g = [0.0, 2e-6, 1e-5]
    cands = sorted(itertools.product(kp_g, ki_g, kd_g))
    scored = []
    for kp, ki, kd in cands:
        t, y = run_average_model((kp, ki, kd), 0.05, VREF, None)
        ys = y[-2000:].mean()
        if not np.isfinite(ys) or abs(y).max() > 60.0:
            continue  # unstable
        m = all_metrics_simple(t, y, VREF)
        if m["overshoot_pct"] > 10.0 or abs(m["ss_error"]) > 0.06:
            continue  # hard constraints
        J = m["overshoot_pct"] / 10.0 + m["t_settle"] / 5e-3 + m["iae"] / 0.05
        scored.append((J, (kp, ki, kd), m))
    scored.sort(key=lambda s: s[0])
    for J, (kp, ki, kd), m in scored:
        pid = DigitalPID(kp, ki, kd, 1.0 / FSW)
        bb = BuckBoost(pid=pid)
        r = bb.run(T_NOM, VREF)
        mm = all_metrics(r, VREF)
        if mm["overshoot_pct"] <= 12.0 and abs(mm["ss_error"]) <= 0.5:
            print(f"tuned: Kp={kp}, Ki={ki}, Kd={kd}  (J={J:.3f})  "
                  f"switched: ov={mm['overshoot_pct']:.1f}%, "
                  f"ts={mm['t_settle']*1e3:.2f} ms, "
                  f"ss_err={mm['ss_error']*1e3:.1f} mV, "
                  f"LC={mm['limit_cycle_amp']*1e3:.1f} mV")
            return (kp, ki, kd), mm
    raise RuntimeError("no candidate passed switched-model validation")


def all_metrics_simple(t, y, vref):
    from sim_core import settling_time, overshoot, steady_error, ripple_pp, iae_itae
    ts, _ = settling_time(t, y)
    ov, _ = overshoot(t, y)
    return dict(t_settle=ts, overshoot_pct=ov, ss_error=steady_error(y, vref),
                ripple_pp=ripple_pp(y), iae=iae_itae(t, y, vref)[0],
                itae=iae_itae(t, y, vref)[1])


# ---------------- simulation batches ----------------

def make_bb(ts_mult, adc_bits, pwm_bits, pid):
    return BuckBoost(vin=VIN, l=L, c=C, fsw=FSW, r=R, ts_mult=ts_mult,
                     adc_bits=adc_bits, pwm_bits=pwm_bits, pid=pid)


def simulate(cfg, t_end, pid, dist=None):
    r = make_bb(cfg["ts"], cfg["adc"], cfg["pwm"], pid).run(
        t_end, VREF, dist=dist)
    t0 = T_STEP if dist is not None else 0.0
    return r, all_metrics(r, VREF, t0=t0)


def vin_step(t, vin, load):
    return (9.0, load) if t >= T_STEP else (vin, load)


def load_step(t, vin, load):
    return (vin, 5.0) if t >= T_STEP else (vin, load)


# ---------------- main ----------------

def main():
    gate = sanity_gate()
    pid_gains, base_sw = tune_controller()
    pid = DigitalPID(*pid_gains, 1.0 / FSW)
    base = make_bb(1, 16, 16, pid)
    r0 = base.run(T_NOM, VREF)
    m0 = all_metrics(r0, VREF)  # ideal digital reference metrics

    rows = []
    def record(kind, cfg, r, m, extra=None):
        rows.append(dict(kind=kind, ts=cfg["ts"], adc=cfg["adc"],
                         pwm=cfg["pwm"],
                         t_settle_ms=m["t_settle"] * 1e3,
                         overshoot_pct=m["overshoot_pct"],
                         ss_error_mV=m["ss_error"] * 1e3,
                         ripple_pp_mV=m["ripple_pp"] * 1e3,
                         limit_cycle_mV=m["limit_cycle_amp"] * 1e3,
                         iae=m["iae"], itae=m["itae"],
                         dcm_hits=m["dcm_cycles"], **(extra or {})))
        return r, m

    # Phase 1: baseline (continuous ideal -- analog PID, no sampling, no
    # quantization) and ideal digital reference (Ts = Tsw, 16-bit ADC/PWM).
    bb_cont = BuckBoost(pid=DigitalPID(*pid_gains, 1e-7), dt_div=200)
    pid_cont = ContinuousPID(*pid_gains)
    r_c = bb_cont.run_continuous(T_NOM, VREF, pid=pid_cont)
    m_c = all_metrics(r_c, VREF)
    record("baseline_continuous", dict(ts=-1, adc=-1, pwm=-1), r_c, m_c)
    r_d, m_d = record("ideal_digital", dict(ts=1, adc=16, pwm=16), r0, m0)

    # Phase 2: individual effects
    for ts in (1, 2, 5):
        r, m = simulate(dict(ts=ts, adc=16, pwm=16), T_NOM, pid)
        record(f"sampling_Ts{ts}x", dict(ts=ts, adc=16, pwm=16), r, m)
    for bits in (12, 10, 8, 6):
        r, m = simulate(dict(ts=1, adc=bits, pwm=16), T_NOM, pid)
        record(f"adc_{bits}bit", dict(ts=1, adc=bits, pwm=16), r, m)
    for bits in (12, 10, 8, 6):
        r, m = simulate(dict(ts=1, adc=16, pwm=bits), T_NOM, pid)
        record(f"pwm_{bits}bit", dict(ts=1, adc=16, pwm=bits), r, m)

    # Phase 3: interaction (3x4x4 = 48)
    for ts, adc, pwm in itertools.product((1, 2, 5), (12, 10, 8, 6), (12, 10, 8, 6)):
        cfg = dict(ts=ts, adc=adc, pwm=pwm)
        r, m = simulate(cfg, T_NOM, pid)
        record("combined", cfg, r, m)

    # Phase 4: robustness -- worst config defined after Phase 3 data
    score_best = None
    for row in rows:
        if row["kind"] != "combined":
            continue
        s = (abs(row["ss_error_mV"] - m0["ss_error"] * 1e3) / max(
                 abs(m0["ss_error"] * 1e3), 1e-3)
             + (row["iae"] - m0["iae"]) / max(m0["iae"], 1e-6)
             + (row["limit_cycle_mV"] - m0["limit_cycle_amp"] * 1e3) / max(
                 m0["limit_cycle_amp"] * 1e3, 1e-3))
        if score_best is None or s > score_best[0]:
            score_best = (s, row)
    if score_best is None:
        raise RuntimeError("no combined rows")
    worst = score_best[1]
    print(f"objective worst config: Ts={worst['ts']}x, "
          f"ADC={worst['adc']}-bit, PWM={worst['pwm']}-bit  (score {score_best[0]:.1f})")
    dist_cases = [("baseline_cont", dict(ts=-1, adc=-1, pwm=-1), bb_cont,
                   pid_cont, True),
                  ("ideal_digital", dict(ts=1, adc=16, pwm=16), base,
                   pid, False),
                  ("worst", dict(ts=worst["ts"], adc=worst["adc"],
                                 pwm=worst["pwm"]),
                   make_bb(worst["ts"], worst["adc"], worst["pwm"], pid),
                   pid, False)]
    dist_results = {}
    for name, cfg, bb, ctrl, is_cont in dist_cases:
        for dname, dist in (("vin_step", vin_step), ("load_step", load_step)):
            if is_cont:
                r = bb.run_continuous(T_DIST, VREF, dist=dist, pid=ctrl)
            else:
                r = bb.run(T_DIST, VREF, dist=dist)
            m = all_metrics(r, VREF, t0=T_STEP)
            record(f"dist_{name}_{dname}", cfg, r, m)
            dist_results[f"{name}_{dname}"] = r

    # persist
    with open(f"{OUT}/results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(f"{OUT}/params.json", "w") as f:
        json.dump(dict(pid_gains=pid_gains, gate=gate,
                       base_metrics={k: v for k, v in m0.items()
                                     if k != "dcm_cycles"}), f, indent=2)
    print(f"saved {len(rows)} rows -> {OUT}/results.csv")
    make_figures(rows, r_c, r_d, pid, base, m0, dist_results)
    write_report_tables(rows, m0)
    return rows


# ---------------- figures ----------------

def style_ax(ax):
    ax.set_ylabel("$V_{out}$ [V]")


def make_figures(rows, r_c, r_d, pid, base, m0, dist_results):
    f = f"{OUT}/figures"

    # 1. baseline transient
    fig, axs = plt.subplots(2, 1, figsize=(6.2, 4.2), sharex=True)
    axs[0].plot(r_c.t * 1e3, r_c.vc, lw=0.8, label="continuous ideal")
    axs[0].plot(r_d.t * 1e3, r_d.vc, lw=0.8, label="ideal digital")
    axs[0].axhline(VREF, color="r", lw=0.6, ls="--")
    axs[0].set_ylabel("$V_{out}$ [V]")
    axs[0].legend(fontsize=8)
    axs[1].plot(r_c.t * 1e3, r_c.d, lw=0.8, label="continuous ideal")
    axs[1].plot(r_d.t * 1e3, r_d.d, lw=0.8, label="ideal digital")
    axs[1].set_ylabel("duty")
    axs[1].set_xlabel("time [ms]")
    axs[0].set_title("Startup transient: continuous ideal vs ideal digital")
    fig.tight_layout()
    fig.savefig(f"{f}/fig1_transient.png")
    plt.close(fig)

    # 2-4. individual sweeps
    def sampling_plot():
        sel = [r for r in rows if r["kind"].startswith("sampling")]
        xs = [r["ts"] for r in sel]
        fig, axs = plt.subplots(1, 3, figsize=(9.5, 2.9))
        axs[0].plot(xs, [r["t_settle_ms"] for r in sel], "o-")
        axs[0].set(xlabel="Ts / Tsw", ylabel="settling [ms]")
        axs[1].semilogy(xs, [abs(r["ss_error_mV"]) for r in sel], "o-")
        axs[1].set(xlabel="Ts / Tsw", ylabel="|ss error| [mV]")
        axs[2].semilogy(xs, [r["limit_cycle_mV"] for r in sel], "o-")
        axs[2].set(xlabel="Ts / Tsw", ylabel="limit-cycle amp [mV]")
        fig.suptitle("Sampling-period sweep (ADC/PWM ideal)")
        fig.tight_layout(); fig.savefig(f"{f}/fig2_sampling.png"); plt.close(fig)
    sampling_plot()
    def bits_plot(kind_prefix, xkey, fname, title):
        sel = [r for r in rows if r["kind"].startswith(kind_prefix)]
        xs = [r[xkey] for r in sel]
        fig, axs = plt.subplots(1, 3, figsize=(9.5, 2.9))
        axs[0].plot(xs, [r["ss_error_mV"] for r in sel], "o-")
        axs[0].set(xlabel="bits", ylabel="ss error [mV]")
        axs[1].plot(xs, [r["limit_cycle_mV"] for r in sel], "o-")
        axs[1].set(xlabel="bits", ylabel="limit-cycle amp [mV]")
        axs[2].plot(xs, [r["iae"] for r in sel], "o-")
        axs[2].set(xlabel="bits", ylabel="IAE [V·s]")
        fig.suptitle(title)
        fig.tight_layout(); fig.savefig(f"{f}/{fname}"); plt.close(fig)
    bits_plot("adc_", "adc", "fig3_adc.png", "ADC-resolution sweep (Ts=Tsw, PWM ideal)")
    bits_plot("pwm_", "pwm", "fig4_pwm.png", "PWM-resolution sweep (Ts=Tsw, ADC ideal)")

    # 5. combined heatmaps (rows = Ts, cols = ss_error / limit-cycle), color
    #    scale normalized per row so each Ts regime is visible
    comb = [r for r in rows if r["kind"] == "combined"]
    fig, axs = plt.subplots(3, 2, figsize=(7.5, 8.5), sharex="col")
    for i, ts in enumerate((1, 2, 5)):
        sub = [r for r in comb if r["ts"] == ts]
        for j, key in enumerate(("ss_error_mV", "limit_cycle_mV")):
            vmax = max(abs(r[key]) for r in sub) or 1.0
            M = np.zeros((4, 4))
            for r in sub:
                M[6 - r["pwm"] // 2, r["adc"] // 2 - 3] = abs(r[key])
            im = axs[i, j].imshow(M, cmap="viridis", vmin=0, vmax=vmax,
                                  aspect="auto")
            axs[i, j].set_xticks(range(4)); axs[i, j].set_xticklabels([12, 10, 8, 6])
            axs[i, j].set_yticks(range(4)); axs[i, j].set_yticklabels([6, 8, 10, 12])
            axs[i, j].set_title(f"Ts={ts}×Tsw — " + ("|ss err| [mV]" if j == 0 else "limit-cycle [mV]"))
            fig.colorbar(im, ax=axs[i, j], fraction=0.046)
        axs[i, 0].set_ylabel("PWM bits")
    fig.tight_layout()
    fig.savefig(f"{f}/fig5_heatmaps.png")
    plt.close(fig)

    # 6. interaction: metrics vs ADC bits per Ts, and vs PWM bits per Ts
    fig, axs = plt.subplots(2, 2, figsize=(8.5, 6))
    for i, (xkey, xlab) in enumerate((("adc", "ADC [bit]"), ("pwm", "PWM [bit]"))):
        for j, mkey in enumerate(("ss_error_mV", "limit_cycle_mV")):
            ax = axs[i, j]
            for ts, c in zip((1, 2, 5), ("tab:blue", "tab:orange", "tab:green")):
                sub = sorted([r for r in comb if r["ts"] == ts], key=lambda r: r[xkey])
                ax.plot([r[xkey] for r in sub], [r[mkey] for r in sub],
                        "o-", color=c, label=f"Ts = {ts}×Tsw")
            ax.set(xlabel=xlab, ylabel=mkey.replace("_", " "))
            if i == 0:
                ax.legend(fontsize=7)
    fig.suptitle("Interaction: does the resolution effect depend on sampling rate?")
    fig.tight_layout()
    fig.savefig(f"{f}/fig6_interaction.png")
    plt.close(fig)

    # 7. disturbances
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.2))
    for j, dname in enumerate(("vin_step", "load_step")):
        ax = axs[j]
        for name in ("baseline_cont", "ideal_digital", "worst"):
            r = dist_results[f"{name}_{dname}"]
            lbl = {"baseline_cont": "continuous ideal",
                   "ideal_digital": "ideal digital",
                   "worst": "worst digital"}[name]
            ax.plot(r.t * 1e3, r.vc, lw=0.7, label=lbl)
        ax.axvline(T_STEP * 1e3, color="k", ls=":", lw=0.7)
        ax.set(xlabel="time [ms]", ylabel="Vout [V]",
               title="Input-voltage step 12→9 V" if dname == "vin_step" else "Load step R 10→5 Ω")
        ax.legend(fontsize=7)
        ax.set_ylim(-16, -6)
    fig.tight_layout()
    fig.savefig(f"{f}/fig7_disturbances.png")
    plt.close(fig)

    # 8. performance map (pass / degraded / fail vs ideal digital reference)
    def classify(r):
        bad = [abs(r["ss_error_mV"]) > max(0.5, 2 * abs(m0["ss_error"] * 1e3)),
               r["limit_cycle_mV"] > 2 * m0["limit_cycle_amp"] * 1e3,
               r["iae"] > 2 * m0["iae"]]
        return sum(bad)
    fig, axs = plt.subplots(1, 3, figsize=(9.5, 2.9))
    for i, ts in enumerate((1, 2, 5)):
        sub = [r for r in comb if r["ts"] == ts]
        M = np.zeros((4, 4))
        for r in sub:
            M[6 - r["pwm"] // 2, r["adc"] // 2 - 3] = classify(r)
        axs[i].imshow(M, cmap="RdYlGn_r", vmin=0, vmax=3, aspect="auto")
        axs[i].set_title(f"Ts = {ts}×Tsw")
        axs[i].set_xticks(range(4)); axs[i].set_xticklabels([12, 10, 8, 6])
        axs[i].set_yticks(range(4)); axs[i].set_yticklabels([6, 8, 10, 12])
        axs[i].set_xlabel("ADC bits"); axs[i].set_ylabel("PWM bits")
    fig.suptitle("Performance map: 0 = within 2× of ideal digital (green) … 3 = fail (red)")
    fig.tight_layout()
    fig.savefig(f"{f}/fig8_perf_map.png")
    plt.close(fig)


def write_report_tables(rows, m0):
    def fmt_mv(r, k):
        return f"{r[k]:.1f}"
    def fmt_s(r, k):
        return f"{r[k]:.2f}"
    def row_str(r):
        return ("| " + f"{r['ts']}×Tsw | {r['adc']} | {r['pwm']} |"
                + f" {fmt_s(r,'t_settle_ms')} ms | {fmt_mv(r,'overshoot_pct')} % |"
                + f" {fmt_mv(r,'ss_error_mV')} mV | {fmt_mv(r,'limit_cycle_mV')} mV |"
                + f" {r['iae']:.3f} V·s | {r['itae']:.3f} V·s² |")
    head = "| Ts | ADC | PWM | settling | overshoot | ss err | limit-cycle | IAE | ITAE |\n|---|---|---|---|---|---|---|---|---|\n"
    with open(f"{OUT}/tables.md", "w") as fh:
        fh.write("## References\n\n" + head)
        for r in rows:
            if r["kind"] in ("baseline_continuous", "ideal_digital") or r["kind"].startswith(("sampling", "adc_", "pwm_")):
                fh.write(row_str(r) + "\n")
        fh.write("\n## Combined matrix (48)\n\n" + head)
        for r in rows:
            if r["kind"] == "combined":
                fh.write(row_str(r) + "\n")
        fh.write("\n## Disturbance tests (metrics over post-step window)\n\n" + head)
        for r in rows:
            if r["kind"].startswith("dist_"):
                fh.write(row_str(r) + "\n")


if __name__ == "__main__":
    main()