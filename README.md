# Quantization-Aware Digital PID Control of an Inverting Buck-Boost Converter on Two Microcontrollers (ESP32-S3 and STM32F103)

**Measured Latency, Effective-Resolution, and Adaptive Update Management**

Hussain Touhid Siddiquee, Syeda Salsabil Islam Ariya, Jasimul Islam Chowdhury — *Leading University, Sylhet*

> Digital control of switching converters is shaped by three implementation-quantized quantities: ADC resolution, DPWM resolution, and sampling period — plus the software stack that executes the loop. This repository contains the simulation-to-hardware study: 79-run switched-model sweep + **cross-platform** closed-loop measurements (ESP32-S3 @240MHz + STM32F103 @72MHz, one shared firmware) + sampling-boundary pacing + lightweight adaptive controller.

- Paper: [`paper.pdf`](paper.pdf) (IEEE Access, 14 pages) · [`paper.docx`](paper.docx) · [`paper.tex`](paper.tex)
- DOI references: 40 entries, all Crossref-verified and cited

---

## What this paper does

An inverting buck-boost converter (Vin=+12 V, Vref=−12 V, L=100 µH, C=220 µF, fsw=100 kHz, R=10 Ω) is studied at three levels:

1. **Continuous (analog) baseline** — ideal PID recomputed every dt, no quantization.
2. **Digital simulation** — velocity-form PID with independent quantization of Ts, ADC (FS=24 V, 12/10/8/6-bit), and PWM (12/10/8/6-bit), plus DCM zero-current boundary. Locates the critical boundary near **3.5–4× Tsw (35–40 µs)** and quantifies super-additive interaction (D_total=1309.2 vs +15%).
3. **Embedded measurement on two platforms** — same controller, same firmware source (`esp32_feas/src/main.cpp`, `#ifdef` hw layer):
   - **ESP32-S3** (240 MHz, HW FPU): LEDC PWM GPIO2 → ADC1 GPIO1
   - **STM32F103 Blue Pill** (72 MHz, no FPU): TIM1 PWM PA8 → ADC1 PA1

Key results:

- **Critical sampling boundary** at 3.5–4× Tsw. Ts=5× fails in every configuration.
- **Hardware bottleneck is acquisition, not compute.** On both MCUs the *Arduino* API dominates — but bare-register access tells the real silicon story:
  | Quantity | ESP32-S3 | STM32F103 Arduino | STM32F103 bare |
  |---|---|---|---|
  | T_ADC | 60.0 µs | 72.8 µs | **4.23 µs** (17× faster) |
  | T_PID int32 | 0.86 µs | 1.31 µs | 1.31 µs |
  | T_PID float+quantizer | 6.8 µs | **231.9 µs** | — |
  | T_PWM | 4.2 µs | 23.9 µs | **5.88 µs** |
  | T_lat (int32) | ≈65 µs | ≈98 µs | **≈11 µs** |
  | Fastest paced loop | 73 µs (7.3× Tsw) | 98 µs (10×) | **26 µs (2.6×)** |
- **Sampling-boundary pacing (mode T).** STM32F103 bare path paces every rung 1×–5× (10–50 µs) and **crosses the 3.5–4× boundary**; the Arduino path on either MCU cannot (floor 73–98 µs).
- **Soft-float tax:** int32 1–2 µs on both; float+`powf` quantizer explodes to 232 µs on the STM32F103 (no FPU) — the deployed firmware uses int32.
- **Noise floor masks quantization** on both MCUs: 1.6–1.9 Vpp ripple at N=64 vs 27–205 mV predicted quantization ripple.
- **Adaptive controller** (QAI + hysteresis, N=8 fast / N=64 slow + ±1-LSB dithering) cuts update frequency by **84%** (SR=0.84) while holding |ess| within ±40 mV (measured on ESP32-S3).
- **True LC plant (5 Ω + 100 µH + 2.2 µF, LM358 follower)** is software-ready (mode T uses `read_vpin_fast`/`pwm_set_fast`) and deferred to hardware validation — `results/figures/circuit_lc_buffer.png`.

---

## Repository structure

```
.
├── paper.tex / paper.pdf / paper.docx   # manuscript (IEEE Access, 14 pages)
├── references.bib                        # 40 Crossref-verified entries
├── sim_core.py                           # switched-state model + velocity PID + metrics + self-checks
├── run_study.py                          # 79-run study: tuning → sweeps → figures → tables
├── run_audits.py                         # boundary + operating-point audits
├── run_predict.py                        # achievable-schedule predictions (Ts >= 60 µs)
├── make_audit_figures.py                 # audit figure composition
├── results/
│   ├── results.csv / params.json / tables.md
│   ├── figures/  fig1_transient … fig8_perf_map
│   │             fig_cross_latency, fig_boundary_pacing, fig_pid_cost
│   │             circuit_lc_buffer.png  ← deferred LC plant
│   └── embedded/  ESP32-S3 logs
│   └── embedded_stm32/  (pending) STM32F103 logs
├── esp32_feas/                           # PlatformIO project (one source, two MCUs)
│   ├── src/main.cpp                      # shared firmware: modes L/R/S/X/A + T; #ifdef hw layer
│   ├── src/bringup.cpp                   # Blue Pill bring-up gate (blink + DWT + ADC timing)
│   ├── platformio.ini                    # [env:esp32s3] [env:bluepill_f103c8] [env:bluepill_bringup]
│   └── results/embedded/                 # captured serial logs + figures
├── ckt_buckboost.png
└── PLAN.md
```

---

## Reproduce

### Simulation (no hardware)

```bash
python -m venv .venv && source .venv/bin/activate
pip install numpy matplotlib

# sanity + full 79-run study
python sim_core.py          # 6 self-checks
python run_study.py         # -> results/results.csv + results/figures/*.png

# audits (boundary sweep, op-point robustness) and prediction sweeps
python run_audits.py
python run_predict.py
python make_audit_figures.py
```

Outputs: `results/results.csv`, `results/tables.md`, `results/figures/fig*.png`.

### Embedded (ESP32-S3 + STM32F103, one source)

**ESP32-S3 wiring:** GPIO2 (LEDC PWM, 100 kHz) shorted to GPIO1 (ADC1). Sensed voltage mapped by `Vout = -Vpin * 24/3.3`.

**STM32F103 wiring:** PA8 (TIM1 PWM, 100 kHz) shorted to PA1 (ADC1) — or via LC plant `PA8→5Ω→100µH→V_filt→LM358→PA1` (see `results/figures/circuit_lc_buffer.png`).

```bash
# requires PlatformIO
pip install platformio

# ESP32-S3
cd esp32_feas
pio run -e esp32s3 -t upload          # flash (240 MHz, monitor 460800)
pio device monitor -e esp32s3          # capture; send L/R/S/X/A/T

# STM32F103 Blue Pill (serial bootloader via CH341)
pio run -e bluepill_f103c8 -t upload  # BOOT0=1, press RESET (serial)
pio device monitor -e bluepill_f103c8  # capture at 115200; send L/R/S/X/A/T
```

Firmware modes (same on both MCUs, via command char):

| Mode | What it measures |
|------|-----------------|
| `L` | Control-loop latency split (T_ADC / T_PID / T_PWM, Arduino vs bare) via `cc()`/DWT |
| `T` | **Sampling-boundary pacing** — exact Ts/Tsw 1×–5× sweep, int32 + bare-register |
| `R` | Effective-resolution sweep (ADC 12/10/8/6 × PWM 8/6/4) |
| `S` | Achievable-schedule ladder N=1/8/32/64 (timing ladder) |
| `X` | Injected computational delay 0–200 µs at fast schedule |
| `A` | Adaptive schedule+dithering (N=8 ↔ N=64, QAI hysteresis, 6 s run) |

Captures are post-processed by `esp32_feas/host/analyze.py` → `results/embedded/fig_v2_*.png`.

### Rebuild the paper

Compiled via TeXFlow/IEEE Access class (needs the bundled Formata fonts). For local LaTeX:

```bash
TFMFONTS="./texflow/data/ieee:" TEXINPUTS="./texflow/data/ieee:" TEXFONTS="./texflow/data/ieee:" \
pdflatex paper && bibtex paper && pdflatex paper && pdflatex paper
```

Or via TeXFlow MCP: `texflow render compile`.

---

## Figures

| # | Description |
|---|-------------|
| Fig. 1 | Power stage (Vin=+12 V, L/C/fsw/R) |
| Fig. 2 | Digital loop: PID + DPWM/ADC quantizers, Vref=−FS/2 half-scale |
| Fig. 3 | Sampling boundary sweep — simulation 1–4× Tsw (lost at 4×) |
| Fig. 4 | ADC effect (half-scale coincidence masks error) |
| Fig. 5 | PWM effect (6-bit → 205 mV limit cycle) |
| Fig. 6 | Performance map (48-run factorial, excellent→unusable) |
| Fig. 7 | Disturbance rejection (Vin 12→9 V, R 10→5 Ω) |
| Fig. 8 | ESP32-S3 latency breakdown (ADC dominates, log scale) |
| Fig. 9 | Effective-resolution sweep (flat — noise masks quantization) |
| Fig. 10–11 | Schedule ladder + injected-latency (IAE 4.5×) |
| Fig. 12 | Adaptive run (SR=0.84) |
| Fig. 13 | **LC plant — deferred** (5Ω/100µH/2.2µF, LM358 follower) |
| Fig. 14 | **Cross-platform latency** — Arduino vs bare (17× ADC) |
| Fig. 15 | **Boundary pacing** — requested vs achieved Ts/Tsw |
| Fig. 16 | **Controller cost** — soft-float tax (int32 vs float) |

---

## Frozen controller gains

Grid search on averaged model, then switched-model validation (constraints: overshoot ≤10%, |ess|≤60 mV, cost J=ov/10+ts/5 ms+IAE/0.05):

```
Kp=0.05, Ki=100.0, Kd=1e-5   (Ts=Tsw=10 µs, velocity-form, d∈[0.05,0.95])
```

Gains are never retuned during the quantization study.

---

## Citation

```bibtex
@article{siddiquee2026quantization,
  title  = {Quantization-Aware Digital PID Control of an Inverting Buck-Boost
            Converter on Two Microcontrollers (ESP32-S3 and STM32F103):
            Measured Latency, Effective-Resolution, and Adaptive Update Management},
  author = {Siddiquee, Hussain Touhid and Ariya, Syeda Salsabil Islam and Chowdhury, Jasimul Islam},
  journal= {IEEE Access},
  year   = {2026}
}
```

---

## Acknowledgment

Leading University, Sylhet — no external funding. Generative AI was used for language editing and code debugging; authors verified all results.

## License

Paper licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
Code: MIT (unless otherwise noted).
