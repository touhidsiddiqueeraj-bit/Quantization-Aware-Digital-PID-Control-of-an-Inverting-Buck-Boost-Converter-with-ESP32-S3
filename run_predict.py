"""Prediction runs for Phase A of the journal expansion.

The ESP32-S3 harness does NOT emulate a buck-boost converter: its plant is a
pure inverting gain (averaged pin voltage of a 100 kHz PWM, mapped by
vout = -vpin*24/3.3) low-passed by an EMA filter. The hardware loop is
therefore a first-order plant, not the switched converter. Two model layers
are needed and kept distinct:

  Layer 1 (buck-boost, design basis): the real converter switched model with
     the paper's frozen gains (Kp/Ki/Kd = 0.05/100/1e-5). This is the design
     basis the paper already reports; here we re-run it at the *achievable*
     schedule scale (Ts/Tsw = 6 ... 680) to produce the journal's prediction
     table: the real converter is catastrophically unstable at every schedule
     the ESP32-S3 can actually sustain, because even the fastest raw loop
     (Ts ~ 62 us) is 6x Tsw and useful oversampled loops exceed 50x Tsw.
     The hardware can therefore never pass through the 3.5-4x critical
     boundary - that is the negative-result narrative.

  Layer 2 (jumper loop) is NOT modelled here: the firmware measurement IS
  the jumper-loop experiment (Modes S/X/A capture it directly). No predictive
  model of the jumper loop is attempted - the noise/EMA dynamics are exactly
  what the hardware reports.

Outputs: results/predict.csv + results/figures/fig_predict.png
"""
import csv
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sim_core import BuckBoost, DigitalPID, all_metrics

VIN, VREF, L, C, FSW, R = 12.0, -12.0, 100e-6, 220e-6, 100e3, 10.0
KP, KI, KD = 0.05, 100.0, 1e-5          # frozen paper gains
T_END = 0.12
OUT = "results"
os.makedirs(OUT + "/figures", exist_ok=True)


def main():
    # schedule, ts_mult, Ts seconds, hardware match (Mode S)
    schedules = [("raw_fast", 6.2, 6.2e-5, "raw ~62 us"),
                 ("n8", 50.0, 5.0e-4, "N=8 ~500 us"),
                 ("n32", 200.0, 2.0e-3, "N=32 ~2 ms"),
                 ("n64", 680.0, 6.84e-3, "N=64 ~6.8 ms")]
    rows = []

    print("=== Layer 1: real buck-boost at achievable schedules (frozen gains) ===")
    print(f"{'schedule':9} {'ts_mult':>7} {'settle(ms)':>10} {'ov%':>7} "
          f"{'ss(V)':>8} {'LC(mV)':>7} {'IAE':>7} {'DCM':>8}")
    for name, tm, ts, match in schedules:
        bb = BuckBoost(vin=VIN, l=L, c=C, fsw=FSW, r=R, ts_mult=tm,
                       adc_bits=8, pwm_bits=8,
                       pid=DigitalPID(KP, KI, KD, ts))
        m = all_metrics(bb.run(T_END, VREF), VREF)
        rows.append(dict(layer="buckboost", schedule=name, ts_mult=tm,
                         hw_match=match, gains="frozen", adc_bits=8,
                         pwm_bits=8, **m))
        print(f"{name:9} {tm:7.0f} {m['t_settle']*1e3:10.1f} "
              f"{m['overshoot_pct']:7.2f} {m['ss_error']:8.3f} "
              f"{m['ripple_pp']*1e3:7.1f} {m['iae']:7.4f} {m['dcm_cycles']:8d}")

    with open(OUT + "/predict.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # fig: settling / LC against Ts/Tsw (log axis), annotate the 3.5-4x
    # boundary the hardware cannot reach
    fig, ax = plt.subplots(1, 2, figsize=(8, 3.2))
    xs = [r["ts_mult"] for r in rows]
    ax[0].plot(xs, [min(r["t_settle"] * 1e3, 120.0) for r in rows],
               "o-", color="tab:red")
    ax[1].plot(xs, [min(r["ripple_pp"] * 1e3, 1e6) for r in rows],
               "s-", color="tab:red")
    for a in ax:
        a.set_xscale("log")
        a.axvspan(3.5, 4.0, alpha=0.15, color="tab:blue",
                  label="sim critical boundary 3.5-4x (unreachable)")
        a.legend(fontsize=8)
    ax[0].set_xlabel("ts/tsw"); ax[0].set_ylabel("settle (ms, cap 120)")
    ax[1].set_xlabel("ts/tsw"); ax[1].set_ylabel("limit cycle (mV, log cap)")
    fig.suptitle("Prediction: real buck-boost at ESP32-S3-achievable schedules")
    fig.tight_layout()
    fig.savefig(OUT + "/figures/fig_predict.png", dpi=150)
    print(f"\nwrote {OUT}/predict.csv and figures/fig_predict.png")


if __name__ == "__main__":
    main()