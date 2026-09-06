# STM32F103 Bare-Register Measurements — Summary

Full serial logs for STM32F103 bare-register mode-T pacing and latency decomposition are ~15 MB and not shipped in this revision due to size. They are available from the corresponding author on request.

This directory ships a **summary** derived from those logs, matching Table 6 (Cross-platform latency decomposition) in paper.tex:

- `v1_stats_stm32.csv` — mean latencies (N=5000 reps, cycle-counter) for ADC (Arduino vs bare), PID (int32/float/pure/float+quantizer), PWM, Tlat, fastest paced loop.
- `mode_T_pacing.csv` — requested vs achieved period for Ts/Tsw = 1..5x on STM32 bare (26µs floor) and ESP32-S3 floor (73µs) for comparison.

Both files are generated from the shared firmware `esp32_feas/src/main.cpp` (bare paths `read_vpin_fast`/`pwm_set_fast`, `mode T`) and are sufficient to reproduce Figs. 14-16 (cross-latency, PID cost, boundary pacing) in the paper.

To reproduce full logs: build `platformio.ini` env `bluepill` (STM32F103, 72MHz, STM32duino), flash, set jumper PA8->PA1, run `pio device monitor --baud 115200`, execute `mode T` via serial (`T` command), capture 5000 reps per quantity.

Contact: Hussain Touhid Siddiquee — Leading University, Sylhet.
