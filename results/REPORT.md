# Buck–Boost Digital Implementation Study — Report

**Authors:** Syeda Salsabil Islam Ariya, Jasimul Islam Chowdhury, Hussain Touhid Siddiquee
**Data:** `results.csv` (79 runs) | `params.json` (tuned gains, sanity gate, baseline) | `figures/` (fig1–fig8)

## 1. System under study

Ideal inverting buck–boost converter: Vin = +12 V, Vref = −12 V, L = 100 µH, C = 220 µF, fsw = 100 kHz, R = 10 Ω, CCM duty D = 0.5 (|vC| = D/(1−D)·Vin), iL,ss = −vC/(R(1−D)) = 2.4 A, ΔiL = Vin·D·Tsw/L = 0.6 A.

Switched-state model (dt = Tsw/200):

- Switch ON:  diL/dt = +Vin/L,      dvC/dt = −vC/(RC)
- Switch OFF: diL/dt = vC/L,        dvC/dt = −(iL + vC/R)/C
- DCM: the OFF branch holds iL at zero (zero-current boundary) until the next ON interval; DCM cycles are counted (`dcm_hits`).

## 2. Controller and quantization

- Velocity-form digital PID; error e[k] = Vmeas[k] − Vref; positive gains; duty saturated to [0.05, 0.95]; no separate integral state (anti-windup inherent).
- Sampling: ADC+controller update every Ts (1×, 2×, 5× Tsw).
- ADC: mid-tread quantizer, full-scale FS = 24 V → Vref = −FS/2 is exactly representable at every tested resolution (12/10/8/6 bits).
- PWM: duty quantized to 2^bits levels (12/10/8/6 bits).
- Continuous baseline: ContinuousPID re-computed every dt (true analog reference).

## 3. Tuning (frozen; never touched after)

Deterministic grid search on the average model — Kp ∈ {0.005, 0.01, 0.02, 0.05}; Ki ∈ {20, 100, 500, 2000}; Kd ∈ {0, 2e-6, 1e-5}; J = ov/10 + ts/5e-3 + IAE/0.05 with constraints ov ≤ 10 %, |ss| ≤ 0.06 V; switched-model validation |ss| ≤ 0.5 V.

**Chosen (frozen): Kp = 0.05, Ki = 100.0, Kd = 1e-5.**

## 4. Study layout (79 simulations)

| Group | Runs | ts | adc | pwm |
|---|---|---|---|---|
| continuous baseline | 1 | — | — | — |
| ideal digital | 1 | 1× | 16 | 16 |
| sampling only | 3 | 1/2/5× | 16 | 16 |
| ADC only | 4 | 1× | 12/10/8/6 | 16 |
| PWM only | 4 | 1× | 16 | 12/10/8/6 |
| combined (all Ts×ADC×PWM) | 48 | 1/2/5× | 12/10/8/6 | 12/10/8/6 |
| disturbances | 6 | on continuous / ideal digital / worst |

Disturbances: Vin step 12→9 V and load step 10→5 Ω at t = 60 ms; metrics computed on the post-step 60 ms window (t0 = T_STEP); ripple/limit-cycle on the final 50 % of the window.

## 5. Results

### 5.1 Continuous vs ideal digital (volts / seconds where noted)

| Metric | Continuous | Ideal digital |
|---|---|---|
| Settling time | 1.96 ms | 2.66 ms |
| Overshoot | 0.07 % | 0.02 % |
| ss error | 0.44 mV | 12.76 mV |
| Limit cycle | 26.7 mV | 39.6 mV |
| IAE | 0.0058 | 0.0109 |
| DCM hits | 0 | 0 |

Sampling only: Ts = 2× → 4.90 ms settling, LC 75.3 mV (still fine). **Ts = 5× fails outright**: never settles, ss error −13.07 V, IAE 0.783, 141 616 DCM cycles.

### 5.2 ADC resolution only (Ts = 1×, PWM = 16-bit)

| bits | ss error (mV) | limit cycle (mV) | settle (ms) |
|---|---|---|---|
| 12 | 12.59 | 53.7 | 2.70 |
| 10 | 14.90 | 57.8 | 2.61 |
| 8 | 0.44 | 26.9 | 2.64 |
| 6 | 0.43 | 27.1 | 3.49 |

The 6/8-bit "improvement" in ss error is a half-scale coincidence: FS = 24 V makes −12.000 V exactly codable, so the lock-on code has zero quantization offset. It costs overshoot (~0.85 %) and settling (3.49 ms at 6 bits). Always state the half-scale caveat — never present this as an ADC benefit.

### 5.3 PWM resolution only

6-bit PWM adds a 204.8 mV limit cycle (vs ~39.5 mV for 12/10/8-bit). Dominant LC mechanism at coarse PWM.

### 5.4 Combined sweep (48 runs) — worst case (objective)

| ts | adc | pwm | settle (ms) | ov % | ss err (V) | LC (mV) | IAE | DCM |
|---|---|---|---|---|---|---|---|---|
| 5× | 8 | 12 | 60.0 (never) | 19.7 | −13.43 | 7330 | 0.789 | 127 075 |

Score = normalized Δ(ss, IAE, LC) vs ideal digital → **worst = Ts=5×, ADC=8-bit, PWM=12-bit (score 1309.2)**. Best 48 = Ts=1×, ADC=6-bit, PWM=8-bit: ss 0.43 mV, LC 27.2 mV, settle 3.66 ms, ov 0.89 %.

Interactions: Ts is dominant; degradation is not additive (5× with 8-bit ADC + 12-bit PWM is worse than any single constraint). Heatmaps in fig5, interaction analysis in fig6.

### 5.5 Disturbances (t = 60 ms)

| config | event | recovery (ms) | ss err | IAE | DCM |
|---|---|---|---|---|---|
| continuous | Vin 12→9 V | 1.43 | −0.002 mV | 0.00136 | 0 |
| continuous | R 10→5 Ω | 0.94 | 0.32 mV | 0.00131 | 0 |
| ideal digital | Vin 12→9 V | 0.92 | 14.9 mV | 0.00170 | 0 |
| ideal digital | R 10→5 Ω | 0.84 | 26.1 mV | 0.00186 | 0 |
| worst (5×,8,12) | Vin 12→9 V | never | −8.89 V | 0.535 | 266 015 |
| worst (5×,8,12) | R 10→5 Ω | never | −5.85 V | 0.353 | 163 360 |

"Recovery" = t_settle − T_STEP (t_settle is absolute; 60.0 ms ⇒ no settling inside the post-step window). The worst configuration loses regulation after either disturbance and chatters in DCM for the whole window.

## 6. Conclusions

1. 16-bit ADC + Ts = 1× restores quasi-analog performance (the 12.8 mV ss error and 39.6 mV LC are the measurable digital floor at these gains).
2. Sampling is the binding constraint: Ts = 5× (200 µs) is unconditionally unstable for this bandwidth, alone and in every combination.
3. Coarse ADC and coarse PWM each add a distinct artifact: PWM ≤ 8-bit → large LC (204.8 mV at 6-bit); ADC 6/8-bit → exact half-scale lock-on (masked ss error, higher overshoot).
4. Worst-case selection is objective: Ts=5×(dominates), then ADC=8-bit coincidental lock-on, then PWM=12-bit; interactions amplify beyond additivity.
5. Disturbance rejection is clean at continuous/ideal digital (~1 ms), catastrophic at the worst config.
### 5.2b Critical sampling boundary (audit 7)

16-bit ADC/PWM sweep of Ts/Tsw = 1.5, 2, 2.5, 3, 3.5, 4:

| Ts/Tsw | settle (ms) | |ss| (mV) | LC (mV) | IAE | DCM |
|---|---|---|---|---|---|
| 1.5 | 3.90 | 0.4 | 29.1 | 0.0147 | 0 |
| 2.5 | 5.87 | 0.7 | 42.9 | 0.0186 | 0 |
| 3 | 6.05 | 12.8 | 72.3 | 0.0164 | 0 |
| 3.5 | 6.38 | 0.5 | 49.5 | 0.0152 | 0 |
| 4 | 59.99 (never) | 12968 | 1590.6 | 0.768 | 101 736 |

→ Practical boundary between 3.5× and 4× Tsw. Settling degrades smoothly (2.66→6.38 ms) then regulation is lost outright.

### 5.4b Interaction metric (audit 9)

D = normalized partial degradation of (|ss|, IAE, LC) vs ideal digital (same formula as worst-case selection). For the worst combined cell (5×, 8-bit ADC, 12-bit PWM):

- D_sampling(5×)=1141.6, D_ADC(8-bit)=0.6, D_PWM(12-bit)=0.0
- additive prediction 1142.2; measured D_total = 1309.2
- **Interaction = +167 (+15%, super-additive)**

### 5.4c Performance map (audit 10)

Classification thresholds: excellent (settle≤4 ms, |ss|≤60 mV, LC≤60 mV, ov≤5%), acceptable (settle≤6 ms, |ss|,LC≤120 mV), marginal, unusable (settle≥50 ms). Per Ts: 1× → 13 excellent / 3 acceptable; 2× → 13 acceptable / 3 marginal; 5× → 16 unusable.

### 5.6 Operating-point robustness (audit 8)

Settling (ms) / limit cycle (mV) at Vin = 10, 12, 15 V (gains frozen, Vref=−12 V):

| config | 10 V | 12 V | 15 V |
|---|---|---|---|
| ideal (1×,16,16) | 2.74 / 67.9 | 2.66 / 39.6 | 2.55 / 28.9 |
| good (1×,10,10) | 2.69 / 28.6 | 2.64 / 57.8 | 2.52 / 80.6 |
| poor (2×,6,6) | 5.40 / 28.9 | 5.11 / 27.7 | 59.3 / 561* (18 DCM) |

* settles only at the window edge. Poor configs are operating-point sensitive; the main conclusions hold.
