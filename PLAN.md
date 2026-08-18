# Journal Expansion Plan — Quantization-Aware Adaptive Digital PID with ESP32-S3

## Target
- Venue: IJPEDS (IAES, IEEE-style two-column). Frame as IEEE open-access style for now.
- Length: ~10-12 pages (current: 8).
- Title (working): "Quantization-Aware Digital PID Control of an Inverting Buck-Boost
  Converter with ESP32-S3: Measured Latency, Effective-Resolution, and Adaptive
  Update Management". Adaptive claim is earned by implementing Mode A for real.

## Locked scope decisions
1. Jumper-emulated plant (GPIO2->GPIO1 in Vref=-12V / FS=24V frame). No physical power stage.
2. Firmware timing (cc()/esp_timer) for T_ADC, T_comp, T_PWM, T_lat, CPU%.
3. Adaptive controller implemented for real; "Adaptive" stays in title.
4. The 3.5-4x Tsw boundary is reframed as a NEGATIVE RESULT: the ADC floor
   (59.7 us raw / 86.6 us mV) makes 10-40 us loops unreachable; useful achievable
   loops are at Ts/Tsw >= 500x, inside the sim's predicted unstable quadrant.
   That is why adaptation is not optional.
5. Measured hardware facts remain ground truth: LEDC 8-bit cap @100 kHz,
   ADC noise sigma ~ 1.65/sqrt(N), compute float 6.7 us / fixed 1.3 us,
   N=64 loop=6.8 ms (0.15 kHz), retuned d=0.507 |ess|<=0.06 V, frozen-gain ripple 0.76.

## Workstreams
- A. Sim prediction runs: achievable-regime sweeps at Ts>=60 us scale + injected
     computational delay (0/0.25/0.5/0.75 equiv) -> fig_predict.* + prediction table.
- B. Firmware v2 (esp32_feas/src/main.cpp):
     L  latency: T_ADC/T_comp/T_PWM/T_lat, CPU% (cc()/esp_timer markers)
     R  effective resolution: bit-truncated ADC 6/8/10/12 x duty-rounded PWM
        (8/6-bit real; 10/12-bit PWM sim-only -> LEDC cap is a finding)
     S  schedule sweep: raw/fast ~62us, N=8/32/64 -> 0.15 kHz, match sim predictions
     X  latency injection: extra compute delay as Tsw-equivalents scaled to achievable range
     A  adaptive PID: QAI from measured ripple/noise; achievable-schedule switching
        (oversampled-slow <-> raw-fast) + dithering with hysteresis; log mode residency,
        f_avg; SR = 1 - f_avg/f_fast vs IAE/settle/ripple/ss-error.
     Core: deterministic timer-ISR loop; fixed + adaptive passes (Cases A-E).
- C. Analysis/figures: extend host/analyze.py + compose_fig.py.
     New figs: latency bars+CPU%, effective-res grid, prediction-vs-observed,
     adaptive mode timeline, SR-vs-performance. Keep fig:ckt/fig:loop/fig:emb.
- D. Paper restructure (document model):
     New title, new intro paragraphs, "Experimental Closed-Loop Platform" section,
     expanded results (latency table, effective-res grid, prediction-vs-observed,
     adaptive results), Discussion/Conclusion rewrite with negative-result narrative,
     IJPEDS abstract/affiliations/keywords metadata, novelty statement,
     6 contributions claim list.
- E. Verify: AI-check gate on new prose (em-dash budget), 0 undefined refs,
     multi-pass compile, vision-pass figures, page count vs target, deliver PDF+docx.

## Sequencing / effort estimate (~2.5-4h active)
1. Snapshot + sim prediction runs   ~15-25 min
2. Firmware v2 (modes L/R/S/X/A+ISR) ~45-75 min  (uncertain block)
3. Flash + capture all modes        ~20-30 min
4. Analysis + figures               ~25-40 min
5. Paper restructure + prose + gate  ~40-60 min
6. Verify + deliver PDF/docx        ~10-15 min

## Risks
- ISR-context ADC reads / esp_timer at rate: may need a re-flash cycle.
- Serial 460800 capture throughput at N=64 schedules.
- Adaptive gains need retuning; keep fixed-mode gains frozen for clean comparison.