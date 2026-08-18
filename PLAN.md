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
## Results recap (measured, all captured to results/embedded/*.log)
- Mode L latency (Esp32-S3 240MHz): ADC raw 60.0 us avg / 148 us max; ADC mV 88.6 us;
  PID int32 0.858 us; PID float 6.78 us; PWM write 4.2 us. CPU% = 1.3% (int32) / 10.4% (float).
- Mode R effective resolution (N=64 schedule): IAE flat 6.8-8.3 across 12/10/8/6-bit ADC
  and 8/6/4-bit PWM -> ADC noise floor (1.6-1.9V pp) masks sim-predicted quantization
  effects (27-205 mV). KEY negative result: hardware noise floor swamps quantization.
- Mode S schedule sweep: ripple 4952(avg1)->1481mV(avg64), ss_err <=0.04V retuned;
  IAE rises with Ts as sim predicts; quantization effects real but noise-masked.
- Mode X latency injection: IAE 0.136->0.61 (4.5x) as injected delay 0->200us; separates
  fast-sampling+latency from slow sampling.
- Mode A adaptive: 87.2% of time slow (91% update reduction), SR=0.84, dither engages on
  QAI>QH near setpoint, set-point step at 2.5s correctly forces fast; 20 transitions.

## [PLANNED, NOT EXECUTED] Expand 20 -> 40 references (lit review)
All 20 new references VERIFIED via Crossref (exact metadata pulled: authors/title/venue/
year/volume/pages) and clickable (doi.org resolves; note 3 entries return 403 to
scripted HEAD = bot-block, confirmed valid via GET). No fabricated/guessed entries —
every DOI below was resolved before inclusion. Look up exact URL for each: append the
DOI to https://doi.org/.

Ordered by literature-review placement (intro P0/P2/P3/P5 + System Model DCM + Experimental):

P0 (quantizers: ADC/DPWM intro)
- R02 Sun, Tan, Siek, ISNE 2010, "Segmented Hybrid DPWM and tunable PID controller
  for digital DC-DC converters", 10.1109/isne.2010.5669174
- R14 Ahmad & Bakkaloglu, ISSCC 2010, "A 300mA 14mV-ripple digitally controlled buck
  converter using frequency domain Σ-Δ ADC and hybrid PWM generator", 10.1109/isscc.2010.5433985
- R18 Chen, Shen, Yan, Tan, Min, Electron. Lett. 2012, "Monolithic digitally controlled
  buck converter with TDC-based ADC sharing delay cells with DPWM", 10.1049/el.2012.2774
- R13 Yao, Ishizuka, Shiya, Soejima, PECI 2018, "48V/5V direct conversion 1MHz DC-DC
  converter with 14ns time-delay digital control", 10.1109/peci.2018.8334973

P2 (theoretical basis: limit cycles, predictive, delay)
- R06 S. White, IEEE TAC 1969, "Quantizer-induced digital controller limit cycles",
  10.1109/tac.1969.1099222  (classic quantizer-limit-cycle antecedent)
- R04 Crovetti, Usmonov, Musolino, Gregoretti, IEEE TPEL 2020, "Limit-Cycle-Free
  Digitally Controlled DC–DC Converters Based on Dyadic Digital PWM",
  10.1109/tpel.2020.2978696
- R11 Foong, Tan, Zheng, ICIA 2011, "Predictive proportional integral controller for
  digital DC-DC converters", 10.1109/iciea.2011.5975991
- R12 He, Xu, Zhou, Chen, Zhou, ICCCAS 2007, "Nonlinear Compensation in Digital
  Controller of Switching DC-DC Converters", 10.1109/icccas.2007.4348278
- R19 Yu, Zhang, Xiong, Wang, Electronics 2023, "A Sliding Mode Controller with Signal
  Transmission Delay Compensation for the Parallel DC/DC Converter's Network Control
  System", 10.3390/electronics13010121

P3 (buck-boost-specific / non-minimum phase)
- R07 Chakraborty, Khaligh, Emadi, Pfaelzer, PESC 2006, "Digital combination of buck and
  boost converters to control a positive buck-boost converter", 10.1109/pesc.2006.1711978
- R08 Chen, Gao, Ye, Int. J. Control Intell. Syst. 2010, "Frequency domain closed-loop
  analysis and sliding mode control of a nonminimum phase buck-boost converter",
  10.2316/journal.201.2010.4.201-2231
- R05 Dordevic & Despotovic, INFOTEH 2021, "Digital Control of Four Switch Synchronous
  Buck-Boost Power Converter based on PIC18F4520", 10.1109/infoteh51037.2021.9400656

System Model (DCM modeling)
- R03 Gong, Xie, Wang, Ning, ICIC 2010, "A Novel Modeling Method of Nonideal Buck-Boost
  Converter in DCM", 10.1109/icic.2010.230

Experimental Closed-Loop Platform (microcontroller/experimental platforms)
- R09 Mohan & Barai, IICPE 2012, "Digital control of zero voltage switching buck
  converter using PIC microcontroller", 10.1109/iicpe.2012.6450428
- R10 Vinodhini, Rajitha, Kumar, ICCPEIC 2014, "dSPACE based 12/24v closed loop boost
  converter for low power applications", 10.1109/iccpeic.2014.6915367
- R17 Suhitha, Harshavardhan, Jaiswal, Kumar, Varalakshmi, ICEES 2024, "IoT Controlled
  DC-DC Converter for Remote Power Monitoring and Control", 10.1109/icees61253.2024.10776888

P5 (adaptive / resource-aware / variable-rate / GaN — the paper's contributions)
- R01 Saysanasongkham, Fukumoto, Arai, Takeuchi, Wada, IFEEC 2013, "An adaptive sampling
  method for a highly reliable digital control power converter", 10.1109/ifeec.2013.6687596
- R16 Zhou & Preindl, APEC 2019, "Variable-Frequency Explicit Model Predictive Control
  of Wide Band Gap DC/DC Converter with Critical Soft Switching", 10.1109/apec.2019.8721846
- R20 Heemels, Johansson, Tabuada, CDC 2012, "An introduction to event-triggered and
  self-triggered control", 10.1109/cdc.2012.6425820  (adaptive-update theory framing)
- R15 Kim, Choi, Kim, Kim, ICCE-Asia 2024, "A GaN-Based Digitally Controlled DC-DC
  Buck Converter with an ADC-Based Digital Controller and Soft-Start", 10.1109/icce-asia63397.2024.10773804

Execution steps (DO NOT run until user approves):
1. Append the 20 @inproceedings/@article/@misc entries above to
   /home/touhid/Documents/texflowmcp/workspace/references.bib with EXACT Crossref
   metadata + \note{Available: \url{https://doi.org/<DOI>}} for clickability.
2. Insert the matching \cite{...} into the corresponding paragraph of the doc model
   (variants/ieee-access.texflow.json) so every new entry is physically cited (no
   orphans; aim for 40 rendered references).
3. Recompile via texflow compile_tex (patched serializer + data/ieee TEXINPUTS +
   pdflatex). Verify: bibitem count == 40, 0 undefined citations, figures still off
   the references page, section order intact.
4. Regenerate paper.pdf + paper.docx, commit.

## [PLANNED, NOT EXECUTED] Additional ESP32-S3 value (no extra hardware)
User has NO RC components — drop pure-RC-plant idea. Zero/new-hardware additions:
- ESP-1 (zero-flash, analysis only): ADC ENOB / INL at operating point from existing
  HTR transfer + noise captures; report achievable ENOB ~10.5 bits at N=1 and the
  ~5 dB/octave oversampling gain -> strengthens the noise-floor-masking claim.
- ESP-2 (zero-flash, re-analysis): limit-cycle/spectral peaks via on-chip/offline FFT
  of the oversampled v stream vs effective resolution (illustrates masking).
- ESP-3 (zero-flash): capture 5-10 adaptive runs at varied EH/EL thresholds to give
  SR-vs-performance distributions / error bars for the central adaptive figure.
- ESP-4 (one firmware tweak + 1 flash): add a second reference-step and a duty
  perturbation in Mode A to exercise dithering re-entry (disturbance emulation).
- ESP-5 (firmware tweak): Optionally upgrade the emulated plant from static gain to a
  first-order low-pass (software-only EMA pole already exists; could add a visible RC
  only if user later provides a cap+resistor — NOT now).
Check with user before flashing; do not exceed the existing jumper emulation.
