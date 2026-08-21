# Quantization-Aware Digital PID Control of an Inverting Buck-Boost Converter with ESP32-S3

**Measured Latency, Effective-Resolution, and Adaptive Update Management**

Hussain Touhid Siddiquee, Syeda Salsabil Islam Ariya, Jasimul Islam Chowdhury — *Leading University, Sylhet*

> Digital control of switching converters is shaped by three implementation-quantized quantities: ADC resolution, DPWM resolution, and sampling period. This repository contains the complete simulation-to-hardware study: 79-run switched-model sweep + ESP32-S3 closed-loop measurements + lightweight adaptive controller.

- Paper: [`paper.pdf`](paper.pdf) (IEEE Access, 11 pages) · [`paper.docx`](paper.docx) · [`paper.tex`](paper.tex)
- DOI references: 40 entries, all Crossref-verified and cited

---

## What this paper does

An inverting buck-boost converter (Vin=+12 V, Vref=−12 V, L=100 µH, C=220 µF, fsw=100 kHz, R=10 Ω) is studied at three levels:

1. **Continuous (analog) baseline** — ideal PID recomputed every dt, no quantization.
2. **Digital simulation** — velocity-form PID with independent quantization of Ts, ADC (FS=24 V, 12/10/8/6-bit), and PWM (12/10/8/6-bit), plus DCM zero-current boundary.
3. **Embedded measurement** — same controller on ESP32-S3 (240 MHz, Arduino core, LEDC PWM GPIO2 → ADC1 GPIO1), timed with the CPU cycle counter.

Key results:

- **Critical sampling boundary** at 3.5–4× Tsw (35–40 µs). Ts=5× fails in every configuration.
- **Super-additive interaction**: D_total=1309.2 vs additive prediction 1142.2 (+15%). Worst case is Ts=5×, ADC 8-bit, PWM 12-bit.
- **Half-scale ADC coincidence**: Vref=−FS/2 is exactly representable, so 8/6-bit ADC masks steady-state error (a measurement artifact, not a benefit).
- **PWM dominates ripple**: 6-bit PWM → 204.8 mV limit cycle vs ~39.5 mV at 12/10/8-bit.
- **Hardware bottleneck is acquisition, not compute**: T_ADC=60.0 µs (raw 12-bit), T_PID=0.86 µs (int32) / 6.8 µs (float32), T_PWM=4.2 µs, T_lat≈65 µs. Fastest loop ≈6× Tsw; N=64 oversampled loop ≈680× Tsw — entirely inside the simulated unstable regime.
- **Noise floor masks quantization** on hardware: 1.6–1.9 Vpp ripple at achievable schedules vs 27–205 mV predicted quantization ripple.
- **Adaptive controller** (QAI + hysteresis, N=8 fast / N=64 slow + ±1-LSB dithering) cuts update frequency by **84%** (SR=0.84) while holding |ess| within ±40 mV.

---

## Repository structure

```
.
├── paper.tex / paper.pdf / paper.docx   # manuscript (IEEE Access)
├── references.bib                        # 40 Crossref-verified entries
├── sim_core.py                           # switched-state model + velocity PID + metrics + self-checks
├── run_study.py                          # 79-run study: tuning → sweeps → figures → tables
├── run_audits.py                         # boundary + operating-point audits
├── run_predict.py                        # achievable-schedule predictions (Ts >= 60 µs)
├── make_audit_figures.py                 # audit figure composition
├── results/
│   ├── results.csv / params.json / tables.md
│   ├── figures/  fig1_transient … fig8_perf_map, circuit_*.png
│   └── embedded/  latency / effective-resolution / schedule / adaptive logs
├── esp32_feas/                           # PlatformIO project (ESP32-S3)
│   ├── src/main.cpp                      # firmware v2: modes L/R/S/X/A
│   ├── platformio.ini
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

Outputs: `results/results.csv`, `results/tables.md`, `results/figures/fig*.png`, `results/REPORT.md`.

### Embedded (ESP32-S3)

Wiring: **GPIO2 (LEDC PWM, 100 kHz) shorted to GPIO1 (ADC1)**. The sensed pin voltage is mapped to the paper frame by `Vout = -Vpin * 24/3.3`.

```bash
# requires PlatformIO
pip install platformio
cd esp32_feas
pio run --target upload          # flash (default 240 MHz, monitor 460800)
pio device monitor --baud 460800  # capture; modes L/R/S/X/A selected in src/main.cpp
```

Firmware modes (edit `MODE` in `src/main.cpp`):

| Mode | What it measures |
|------|-----------------|
| `L` | Control-loop latency split (T_ADC / T_PID / T_PWM) via `cc()` |
| `R` | Effective-resolution sweep (ADC 12/10/8/6 × PWM 8/6/4) |
| `S` | Achievable-schedule ladder N=1/8/32/64 (73 µs → 6.84 ms) |
| `X` | Injected computational delay 0–200 µs at fast schedule |
| `A` | Adaptive schedule+dithering (N=8 ↔ N=64, QAI hysteresis, 6 s run) |

Captures are post-processed by `esp32_feas/host/analyze.py` → `results/embedded/fig_v2_*.png`.

### Rebuild the paper

Compiled via TeXFlow (IEEE Access class). For local LaTeX:

```bash
pdflatex paper && bibtex paper && pdflatex paper && pdflatex paper
```

Or via TeXFlow MCP: `texflow render compile`.

---

## Figures

| # | Description |
|---|-------------|
| Fig. 1 | Power stage (Vin=+12 V, L/C/fsw/R) |
| Fig. 2 | Digital loop: PID + DPWM/ADC quantizers, Vref=−FS/2 half-scale |
| Fig. 3 | Sampling boundary sweep (1–4× Tsw, regulation lost at 4×) |
| Fig. 4 | ADC effect (half-scale coincidence masks error) |
| Fig. 5 | PWM effect (6-bit PWM → 205 mV limit cycle) |
| Fig. 6 | Performance map (48-run factorial, excellent→unusable) |
| Fig. 7 | Disturbance rejection (Vin 12→9 V, R 10→5 Ω) |
| Fig. 8 | Measured latency breakdown (ADC dominates, log scale) |
| Fig. 9 | Effective-resolution sweep (flat — noise masks quantization) |
| Fig. 10–11 | Schedule ladder + injected-latency (IAE 4.5×) |
| Fig. 12 | Adaptive run (startup + Vref −12→−6 V step, SR=0.84) |

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
            Converter with {ESP32-S3}: Measured Latency, Effective-Resolution,
            and Adaptive Update Management},
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
