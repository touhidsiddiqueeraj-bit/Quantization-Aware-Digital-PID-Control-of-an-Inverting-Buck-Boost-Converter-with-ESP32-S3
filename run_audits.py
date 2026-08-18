"""Audit-7/8 runs: critical sampling boundary sweep + operating-point robustness.

Appends to results/results.csv with kinds `boundary_Ts{n}` and `op_{cfg}_{vin}`.
Runs only the new experiments (does not re-run the 67-run matrix).
"""
import csv, json
import numpy as np
from sim_core import BuckBoost, DigitalPID, all_metrics

VIN, VREF, R = 12.0, -12.0, 10.0
T_NOM, T_STEP = 0.06, 0.06
OUT = "results"
gains = json.load(open(f"{OUT}/params.json"))["pid_gains"]
PID = DigitalPID(*gains, 1.0 / 100e3)


def run(ts_mult, vin):
    bb = BuckBoost(vin=vin, ts_mult=ts_mult, adc_bits=16, pwm_bits=16, pid=PID)
    r = bb.run(T_NOM, VREF)
    return r, all_metrics(r, VREF)


def main():
    rows = list(csv.DictReader(open(f"{OUT}/results.csv")))
    if rows:
        fieldnames = sorted(set(rows[0].keys()) | {"vin"})
    else:
        fieldnames = []

    for tm in (1.5, 2, 2.5, 3, 3.5, 4):
        r, m = run(tm, VIN)
        rows.append(dict(kind=f"boundary_Ts{tm}", ts=tm, adc=16, pwm=16,
                         t_settle_ms=m["t_settle"] * 1e3,
                         overshoot_pct=m["overshoot_pct"],
                         ss_error_mV=m["ss_error"] * 1e3,
                         ripple_pp_mV=m["ripple_pp"] * 1e3,
                         limit_cycle_mV=m["limit_cycle_amp"] * 1e3,
                         iae=m["iae"], itae=m["itae"], dcm_hits=m["dcm_cycles"],
                         vin=VIN))
        print(f"Ts={tm}x: settle {m['t_settle']*1e3:.2f} ms | ss {m['ss_error']*1e3:.1f} mV | LC {m['limit_cycle_amp']*1e3:.1f} mV | IAE {m['iae']:.4f} | dcm {m['dcm_cycles']}")

    for cfg, name in [((1, 16, 16), "ideal"), ((1, 10, 10), "good"), ((2, 6, 6), "poor")]:
        ts, adc, pwm = cfg
        for vin in (10.0, 15.0):
            bb = BuckBoost(vin=vin, ts_mult=ts, adc_bits=adc, pwm_bits=pwm, pid=PID)
            r = bb.run(T_NOM, VREF)
            m = all_metrics(r, VREF)
            rows.append(dict(kind=f"op_{name}_{int(vin)}", ts=ts, adc=adc, pwm=pwm,
                             t_settle_ms=m["t_settle"] * 1e3,
                             overshoot_pct=m["overshoot_pct"],
                             ss_error_mV=m["ss_error"] * 1e3,
                             ripple_pp_mV=m["ripple_pp"] * 1e3,
                             limit_cycle_mV=m["limit_cycle_amp"] * 1e3,
                             iae=m["iae"], itae=m["itae"], dcm_hits=m["dcm_cycles"],
                             vin=vin))
            print(f"op {name} @Vin={vin}V: settle {m['t_settle']*1e3:.2f} ms | ss {m['ss_error']*1e3:.1f} mV | LC {m['limit_cycle_amp']*1e3:.1f} mV | IAE {m['iae']:.4f} | ov {m['overshoot_pct']:.2f}% | dcm {m['dcm_cycles']}")

    with open(f"{OUT}/results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    print(f"saved {len(rows)} rows")


if __name__ == "__main__":
    main()