// ESP32-S3 embedded-feasibility harness for the buck-boost digital PID study.
//
// Wiring: GPIO2 (LEDC PWM, ~100 kHz) jumper-shorted to GPIO1 (ADC1_CH0). The
// controller drives PWM duty d; the averaged pin voltage d*V_LOGIC is the
// sensed "output voltage", mapped into the paper's Vref=-12 V / FS=24 V frame:
//   v_out = Vpin * 24/3.3 - 24   (Vpin=0 -> -24 V, 1.65 V -> -12 V, 3.3 V -> 0 V)
// so Vref = -FS/2 exactly -- the paper's half-scale-coincidence operating point.
//
// Modes (serial: one char + drain):
//   B   compute-pipeline benchmark (float + int32 Q-fixed), CSV out
//   C   closed loop over the jumper at Ts = {1,2,5}*Tsw, float+fixed, CSV out
//   H   hardware characterization (LEDC resolution @ fsw, ADC read latency)
//
// All results stream as "CSV,..." lines at 460800 baud.
#include <Arduino.h>
#include <algorithm>
#ifdef ARDUINO_ARCH_ESP32
#include <esp_timer.h>
#define PLATFORM_NAME "esp32s3"
#else
#define PLATFORM_NAME "stm32f103"
#endif

#ifdef ARDUINO_ARCH_ESP32
#define PWM_PIN   2
#define ADC_PIN   1         // ADC1_CH0
#else
#define PWM_PIN   PA8       // TIM1_CH1
#define ADC_PIN   PA1       // ADC1_IN1
#endif
#define V_LOGIC   3.3f
#define FSW       100000.0f
#define TSW_US    10.0f
#define ADC_FS    24.0f
#define VREF      -12.0f
#define D_MIN     0.05f
#define D_MAX     0.95f
#define KP        0.05f
#define KI        100.0f
#define KD        1e-5f
#define RES_BITS  8          // working DPWM resolution at ~100 kHz (Mode H)

static float MHZ = 240.0f;
#ifdef ARDUINO_ARCH_ESP32
static inline uint32_t cc() { uint32_t c; asm volatile("rsr %0, ccount" : "=r"(c)); return c; }
#else
static inline uint32_t cc() { return dwt_getCycles(); }
#endif
static inline float ns_span(uint32_t dt) { return dt * 1000.0f / MHZ; }

// ---------------- paper arithmetic: float ----------------
struct PIDF {
    float kp, ki, kd, ts;
    float e1 = 0, e2 = 0, d = D_MIN;
    void reset() { e1 = e2 = 0; d = D_MIN; }
    void step(float meas, float vref) {
        float e = meas - vref;
        float du = kp * (e - e1) + ki * ts * e + (kd / ts) * (e - 2 * e1 + e2);
        e2 = e1; e1 = e;
        d = constrain(d + du, D_MIN, D_MAX);
    }
};
static float fqabs(float x, float bits) {         // ADC: mid-tread, FS=24 V
    float q = powf(2.f, bits);
    return roundf(x / ADC_FS * q) / q * ADC_FS;
}
static float fqduty(float d, float bits) {        // DPWM: 2^bits levels over [0,1]
    float q = powf(2.f, bits);
    return roundf(d * q) / q;
}

// ---------------- int32 fixed-point (error Q10 x1024, duty Q15 x32768) ----
#define E_Q 1024
#define D_Q 32768
struct PIDQ {
    int32_t kp24, kits24, kdts24;                 // gains * 2^24
    int32_t e1 = 0, e2 = 0, d_q15 = (int32_t)(D_MIN * D_Q);
    void reset() { e1 = e2 = 0; d_q15 = (int32_t)(D_MIN * D_Q); }
    void step(int32_t e) {                        // e in Q10
        int64_t acc = (int64_t)kp24 * (e - e1)
                    + (int64_t)kits24 * e
                    + (int64_t)kdts24 * (e - 2 * e1 + e2);
        e2 = e1; e1 = e;
        int32_t du = (int32_t)(acc >> 19);
        int64_t nd = (int64_t)d_q15 + du;
        d_q15 = (int32_t)constrain(nd, (int64_t)(D_MIN * D_Q), (int64_t)(D_MAX * D_Q));
    }
    float duty() { return (float)d_q15 / D_Q; }
};
static inline int32_t to_q10(float v) { return (int32_t)(v * E_Q); }
static int32_t q1q10(int32_t x, float step) {      // mid-tread quantizer, step volts (Q10)
    int64_t sq = (int64_t)(step * E_Q);
    int64_t n = (x * 2 + (x >= 0 ? sq : -sq)) / (2 * sq);
    return (int32_t)(n * sq);
}
static int32_t q1duty(int32_t d_q15, int32_t nlev) {         // DPWM Q15 -> Q15
    int64_t n = ((int64_t)d_q15 * nlev * 2 + (d_q15 >= 0 ? 1 : -1)) / (2 * (int64_t)D_Q);
    return (int32_t)(n * (int64_t)D_Q / nlev);
}

// ---------------- hardware (PWM + ADC) ----------------
static uint8_t g_res_bits = 8;
static uint8_t g_ch = 0;
static uint8_t g_pin = PWM_PIN;
#ifdef ARDUINO_ARCH_ESP32
static void pwm_init(uint8_t res_bits) {
    g_res_bits = res_bits;
    ledcSetup(g_ch, (uint32_t)FSW, res_bits);
    ledcAttachPin(PWM_PIN, g_ch);
    ledcWrite(g_ch, 0);
}
static void pwm_set(float d) {
    uint32_t maxd = (1u << g_res_bits) - 1;
    ledcWrite(g_ch, (uint32_t)(d * maxd));
}
static float read_vpin() { return (float)analogReadMilliVolts(ADC_PIN) / 1000.0f; }
static float read_vpin_raw() { return (float)analogReadRaw(ADC_PIN) * 3.3f / 4095.0f; } // 12-bit @ 11dB
// ESP32: no separate bare-register path (LEDC/analogRead are already the
// native paths); fast == raw so mode T/L run unchanged on both platforms.
static void adc_fast_init() { }
static inline float read_vpin_fast() { return read_vpin_raw(); }
static inline void pwm_set_fast(float d) { pwm_set(d); }
#else
static void pwm_init(uint8_t res_bits) {
    g_res_bits = res_bits;
    analogWriteResolution(res_bits);
    analogWriteFrequency((uint32_t)FSW);   // core API: global freq, all timers
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, 0);
}
static void pwm_set(float d) {
    uint32_t maxd = (1u << g_res_bits) - 1;
    analogWrite(PWM_PIN, (uint32_t)(d * maxd));
}
static float read_vpin() {
    // ponytail: no per-sample hw calibration on F103; raw*3.3/4095 == the
    // "calibrated" path (T_ADC(mV) parity line still measured for the table)
    return (float)analogRead(ADC_PIN) * 3.3f / 4095.0f;
}
static float read_vpin_raw() { return (float)analogRead(ADC_PIN) * 3.3f / 4095.0f; } // 12-bit native

// ---- bare-register fast paths: the Arduino layer costs ~80 us/read and
// ~30 us/write (pinmap + HAL reconfig per call). Direct ADC1/TIM1 access is
// ~1-2 us / ~0.1 us and is what makes the Ts/Tsw boundary rungs reachable.
static uint8_t g_adc_ch = 1;   // PA1 = ADC1_IN1 (channel 1), hardwired
static void adc_fast_init() {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    pinMode(ADC_PIN, INPUT_ANALOG);
    ADC1->CR2 |= ADC_CR2_ADON;          // power on (stm32duino may have it on)
    ADC1->CR2 |= ADC_CR2_RSTCAL;        // reset calibration
    while (ADC1->CR2 & ADC_CR2_RSTCAL) { }
    ADC1->CR2 |= ADC_CR2_CAL;           // calibrate
    while (ADC1->CR2 & ADC_CR2_CAL) { }
    // regular-channel software start: EXTTRIG on, EXTSEL=111 (SWSTART)
    ADC1->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL;
    ADC1->CR2 &= ~ADC_CR2_CONT;         // single-shot
}
static inline float read_vpin_fast() {
    __HAL_RCC_ADC1_CLK_ENABLE();
    ADC1->SQR1 = 0;                     // sequence length = 1
    ADC1->SQR3 = g_adc_ch;              // channel 1st in sequence
    uint8_t shift = g_adc_ch * 3;       // SMPR2 covers channels 0-9
    ADC1->SMPR2 = (ADC1->SMPR2 & ~(7u << shift)) | (1u << shift); // 1.5 cyc
    if (!(ADC1->CR2 & ADC_CR2_ADON)) {  // powered down (HAL stops it): the
        ADC1->CR2 |= ADC_CR2_ADON;      // first ADON only powers up (t_STAB)
        for (volatile int i = 0; i < 400; i++) { }  // ~4 us stabilization
    }
    // rebuild CR2 for a clean software-start shot: HAL leaves DMA/CONT/trigger
    // bits behind; clear them, keep power + SWSTART trigger source, then fire.
    uint32_t cr2 = ADC1->CR2;
    cr2 &= ~(ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_CAL | ADC_CR2_RSTCAL
             | ADC_CR2_JEXTTRIG | ADC_CR2_JEXTSEL);
    cr2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL;   // EXTSEL=111 -> SWSTART src
    ADC1->CR2 = cr2;
    ADC1->CR2 |= ADC_CR2_SWSTART;       // fire regular-channel conversion
    uint32_t to = 5000;                 // EOC timeout: fail safe, never hang
    while (!(ADC1->SR & ADC_SR_EOC) && --to) { }
    if (to == 0) {                      // one-shot diagnostic on first timeout
        static bool dumped = false;
        if (!dumped) {
            dumped = true;
            Serial.printf("CSV,ADCDBG,cr2=%lx sr=%lx dr=%lx apb2en=%lx\n",
                          (unsigned long)ADC1->CR2, (unsigned long)ADC1->SR,
                          (unsigned long)ADC1->DR, (unsigned long)RCC->APB2ENR);
        }
    }
    return (float)ADC1->DR * 3.3f / 4095.0f;
}
static inline void pwm_set_fast(float d) {
    // TIM1 CH1 (PA8): direct CCR write. Scale by the timer's live ARR --
    // Arduino chose prescaler+period for ~100 kHz; we keep its config and
    // only move the compare point (frequency parity with the Arduino path).
    TIM1->CCR1 = (uint32_t)(d * TIM1->ARR);
}
#endif
static inline float vout_of(float vpin) { return -vpin * ADC_FS / V_LOGIC; }
// vpin=0 -> 0 V, vpin=1.65 (d=0.5) -> -12 V = VREF, vpin=3.3 -> -24 V
// inverting, matching the buck-boost: higher duty -> more negative output.

// average N raw ADC reads -> mean pin voltage (software RC filter equivalent).
// delayMicroseconds(dither) decorrelates each read from the 100 kHz PWM phase.
static float read_vpin_avg(int n) {
    float s = 0;
    for (int i = 0; i < n; i++) {
        uint32_t d = 0;
        // dither: odd/i-derived delay so consecutive reads hit different PWM phases
        if (i >= 1) { d = 13u * i % 97u; delayMicroseconds(d); }
        s += read_vpin_raw();
    }
    return s / n;
}

// ---------------- mode B: compute-pipeline benchmark ----------------
static void mode_bench() {
    const int NIT = 20000;
    static uint32_t vs[512];

    // float pipeline
    for (int t = 0; t < 3; t++) {
        float ts = (t == 0) ? 10.f : (t == 1) ? 20.f : 50.f;
        for (int ab : {12, 10, 8, 6}) {
            for (int pb : {12, 10, 8, 6}) {
                PIDF pid; pid.kp = KP; pid.ki = KI; pid.kd = KD; pid.ts = ts * 1e-6f;
                uint32_t mn = 0xFFFFFFFF, mx = 0; uint64_t sm = 0, vn = 0;
                for (int i = 0; i < NIT; i++) {
                    float meas = -12.0f + 0.5f * sinf(i * 0.001f);
                    uint32_t a = cc();
                    float mq = fqabs(meas, ab);
                    pid.step(mq, VREF);
                    (void)fqduty(pid.d, pb);
                    uint32_t dt = cc() - a;
                    if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt;
                    if (vn < 512) vs[vn++] = dt;
                }
                std::sort(vs, vs + 512);
                uint32_t p99 = vs[(int)(vn * 0.99)];
                Serial.printf("CSV,B,%d,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f\n",
                    t, ts, ab, pb, ns_span(mn), sm * 1000.0f / MHZ / NIT, ns_span(mx), ns_span(p99));
            }
        }
    }
    // fixed pipeline + duty deviation vs float (same input stream)
    for (int t = 0; t < 3; t++) {
        float ts = (t == 0) ? 10.f : (t == 1) ? 20.f : 50.f;
        for (int ab : {12, 10, 8, 6}) {
            for (int pb : {12, 10, 8, 6}) {
                PIDQ pq; pq.kp24 = (int32_t)(KP * (1 << 24));
                pq.kits24 = (int32_t)(KI * ts * 1e-6f * (1 << 24));
                pq.kdts24 = (int32_t)((KD / (ts * 1e-6f)) * (1 << 24));
                PIDF pf; pf.kp = KP; pf.ki = KI; pf.kd = KD; pf.ts = ts * 1e-6f;
                float stepA = ADC_FS / powf(2, ab);
                int32_t nlev = 1 << pb;
                uint32_t mn = 0xFFFFFFFF, mx = 0; uint64_t sm = 0, vn = 0;
                float maxdev = 0;
                for (int i = 0; i < NIT; i++) {
                    float meas = -12.0f + 0.5f * sinf(i * 0.001f);
                    uint32_t a = cc();
                    int32_t mq = q1q10(to_q10(meas), stepA);
                    pq.step(mq - to_q10(VREF));
                    int32_t dq = q1duty(pq.d_q15, nlev);
                    uint32_t dt = cc() - a;
                    if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt;
                    if (vn < 512) vs[vn++] = dt;
                    float df = fqduty(pf.d, pb);
                    float dev = fabsf((float)dq / D_Q - df);
                    if (dev > maxdev) maxdev = dev;
                    // feed same measurement into float controller (after quantization)
                    float mqf = fqabs(meas, ab);
                    pf.step(mqf, VREF);
                }
                std::sort(vs, vs + (int)vn);
                Serial.printf("CSV,BF,%d,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.4f\n",
                    t, ts, ab, pb, ns_span(mn), sm * 1000.0f / MHZ / NIT, ns_span(mx),
                    ns_span(vs[(int)(vn * 0.99)]), maxdev);
            }
        }
    }
    Serial.println("CSV,BDONE");
}

// ---------------- mode C: closed loop over the jumper ----------------
// retune=true uses bandwidth-limited gains (small Ki, no D) matched to the
// ADC-bound sample rate; retune=false ports the paper's frozen gains as-is.
// ponytail: v1 mode, superseded by S/X; compiled out on F103 (24 KB of
// static log buffers would not fit the 20 KB RAM)
#ifdef ARDUINO_ARCH_ESP32
static void mode_closed(int ts_mult, int avg_n, bool retune) {
    float ts = TSW_US * ts_mult;
    const int SENSE_BITS = 8, DUTY_BITS = 8;
    float stepA = ADC_FS / 256.0f;
    int32_t nlev = 1 << DUTY_BITS;
    int TARGET = avg_n <= 8 ? 2000 : 500;
    static float log_t[2000], log_m[2000], log_d[2000];
    const float ALPHA = 0.08f;   // EMA smoothing of the ADC average (anti-alias)
    for (int pass = 0; pass < 2; pass++) {
        pwm_init(RES_BITS);

        // phase 1: measure the achieved step period (ADC-bound, independent of ts_mult)
        uint32_t t0 = micros();
        for (int i = 0; i < 10; i++) { vout_of(read_vpin_avg(avg_n)); }
        uint32_t t1 = micros();
        float ts_meas_us = (float)(t1 - t0) / 10.0f;
        float ts_att = ts_meas_us * 1e-6f;          // actual sample period
        float kp = KP, ki = KI, kd = KD;
        if (retune) { kp = 0.02f; ki = 0.5f; kd = 0.0f; }

        PIDF pf; pf.kp = kp; pf.ki = ki; pf.kd = kd; pf.ts = ts_att;
        PIDQ pq; pq.kp24 = (int32_t)(kp * (1 << 24));
        pq.kits24 = (int32_t)(ki * ts_att * (1 << 24));
        pq.kdts24 = (int32_t)((kd / ts_att) * (1 << 24));

        uint32_t mn = 0xFFFFFFFF, mx = 0; uint64_t sm = 0, vn = 0;
        static uint32_t vs[2048];
        uint32_t done = 0;
        uint64_t t_start = micros();
        float filt = 0; bool filt_init = false;
        while (done < TARGET) {
            uint32_t a = cc();
            float meas_raw = vout_of(read_vpin_avg(avg_n));
            float meas = filt_init ? ALPHA * meas_raw + (1 - ALPHA) * filt : meas_raw;
            filt = meas; filt_init = true;
            float d_out;
            if (pass == 0) {
                float mq = fqabs(meas, SENSE_BITS);
                pf.step(mq, VREF);
                d_out = fqduty(pf.d, DUTY_BITS);
            } else {
                int32_t mq = q1q10(to_q10(meas), stepA);
                pq.step(mq - to_q10(VREF));
                d_out = (float)q1duty(pq.d_q15, nlev) / D_Q;
            }
            pwm_set(d_out);
            uint32_t dt = cc() - a;
            if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt;
            if (vn < 2048) vs[vn++] = dt;
            log_t[done] = (float)(micros() - t_start) / 1e3f; // ms
            log_m[done] = meas;
            log_d[done] = d_out;
            done++;
        }
        // analysis
        int n = done;
        double t_end = (double)log_t[n - 1];
        double rate_khz = (double)(n - 1) / (t_end * 1e-3) / 1e3;
        // settled window = last 20%
        int sw = n - (n / 5);
        double ss_mean = 0, ss_ripple = 0, d_mean = 0, d_ripple = 0;
        float ss_min = 1e9f, ss_max = -1e9f, dm_min = 1e9f, dm_max = -1e9f;
        for (int i = sw; i < n; i++) {
            ss_mean += log_m[i]; d_mean += log_d[i];
            if (log_m[i] < ss_min) ss_min = log_m[i];
            if (log_m[i] > ss_max) ss_max = log_m[i];
            if (log_d[i] < dm_min) dm_min = log_d[i];
            if (log_d[i] > dm_max) dm_max = log_d[i];
        }
        int wn = n - sw;
        ss_mean /= wn; d_mean /= wn;
        ss_ripple = (ss_max - ss_min) * 1000.0f;          // mV
        d_ripple = dm_max - dm_min;                        // duty
        double ss_err = ss_mean - VREF;                    // V
        // settle: reverse scan for last exit from [VREF-0.5, VREF+0.5]
        double settle_ms = -1;
        for (int i = n - 1; i >= 0; i--) {
            if (log_m[i] < VREF - 0.5 || log_m[i] > VREF + 0.5) { settle_ms = log_t[i]; break; }
        }
        std::sort(vs, vs + (int)vn);
        uint32_t p99 = vs[(int)(vn * 0.99)];
        Serial.printf("CSV,C,%d,%d,%.1f,%.3f,%.0f,%.0f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.1f,%d\n",
            ts_mult, pass, ts, ss_err, ss_ripple, settle_ms,
            d_mean, d_ripple, rate_khz,
            ns_span(mn), sm * 1000.0f / MHZ / vn, ns_span(mx), ns_span(p99), avg_n, ts_meas_us,
            retune ? 1 : 0);
        Serial.printf("CSV,CT,%d,%d,%d,%d,%d\n", ts_mult, pass, n, avg_n, retune ? 1 : 0); // time series follows
        for (int i = 0; i < n; i++)
            Serial.printf("CSV,CT,%d,%d,%d,%d,%d,%.3f,%.4f,%.4f\n", ts_mult, pass, i, avg_n, retune ? 1 : 0, log_t[i], log_m[i], log_d[i]);
    }
    pwm_set(0.5f);
}
#endif // ARDUINO_ARCH_ESP32 (mode C)

// ---------------- mode H: hardware characterization ----------------
static void mode_hw() {
    // PWM achievable frequency per resolution at a 100 kHz request
    for (int bits = 1; bits <= 12; bits++) {
#ifdef ARDUINO_ARCH_ESP32
        uint32_t actual = ledcSetup(g_ch, (uint32_t)FSW, (uint8_t)bits);
        Serial.printf("CSV,H,%d,%u,%d\n", bits, actual, (actual ? 1 : 0));
#else
        // stm32duino: analogWriteFrequency is global; probe by reconfiguring
        // TIM1 directly: period = F_CPU/fsw, max res bits where 2^bits-1 fits
        uint32_t period = (uint32_t)(F_CPU / FSW);   // 720 @ 72 MHz / 100 kHz
        int ok = ((1u << bits) - 1) <= period;
        Serial.printf("CSV,H,%d,%lu,%d\n", bits, (unsigned long)period, ok);
#endif
    }
    pwm_init(RES_BITS); // restore working config
    // ADC read latency (both paths)
    uint32_t mn = 0xFFFFFFFF, mx = 0, mnr = 0xFFFFFFFF, mxr = 0;
    uint64_t sm = 0, smr = 0;
    for (int i = 0; i < 3000; i++) {
        uint32_t a = cc(); (void)read_vpin(); uint32_t dt = cc() - a;
        if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt;
        a = cc(); (void)read_vpin_raw(); dt = cc() - a;
        if (dt < mnr) mnr = dt; if (dt > mxr) mxr = dt; smr += dt;
    }
    Serial.printf("CSV,HADC,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0f / MHZ / 3000.0f, ns_span(mx));
    Serial.printf("CSV,HADCRAW,%.2f,%.2f,%.2f\n", ns_span(mnr), smr * 1000.0f / MHZ / 3000.0f, ns_span(mxr));
    // ADC mean-noise floor vs oversampling N at fixed duty 0.5 (worst case)
    pwm_init(RES_BITS);
    pwm_set(0.5f);
    delay(5);
    for (int N : {1, 4, 8, 16, 32, 64, 128}) {
        float s = 0, s2 = 0; int m = 40;
        for (int k = 0; k < m; k++) {
            float v = 0;
            for (int j = 0; j < N; j++) v += read_vpin_raw();
            v /= N;
            s += v; s2 += v * v;
        }
        float mean = s / m;
        float sd = sqrtf(s2 / m - mean * mean);
        Serial.printf("CSV,HN,%d,%.5f,%.5f\n", N, mean, sd);
    }
    // fixed-duty -> measured pin voltage transfer (averaged)
    for (int i = 5; i <= 95; i += 5) {
        float d = i / 100.0f;
        pwm_set(d);
        delay(5);
        float s = 0; int n = 32;
        for (int k = 0; k < n; k++) s += read_vpin_raw();
        Serial.printf("CSV,HTR,%d,%.4f,%.4f\n", i, s / n, vout_of(s / n));
    }
    Serial.println("CSV,HDONE");
}

// ---------------- v2 modes: L / R / S / X / A / T ----------------
#ifdef ARDUINO_ARCH_ESP32
#define LOGN 8000
#else
// ponytail: F103 has 20 KB RAM; 4 x LOGN x 4 B of log + stack must fit.
// 1000 overflowed the linker by 400 B -> 900 (14.4 KB) with headroom.
#define LOGN 900
#endif
float g_t[LOGN], g_v[LOGN], g_d[LOGN], g_st[LOGN];   // shared log buffers
// Note: mode_closed() (v1) already measures the achieved ADC-bound sample
// period; the v2 modes reuse the same read path but add (a) a latency
// breakdown, (b) effective-resolution sweeps, (c) an explicit schedule
// sweep at the achievable periods, (d) injected computational latency, and
// (e) an adaptive schedule/dithering controller.

// shared loop metrics from a run: fill arrays, caller prints
struct LoopStats {
    double ts_meas_us;        // measured mean period
    double rate_khz;
    float ss_mean, ss_min, ss_max, d_mean, d_min_s, d_max_s;
    double ss_err, settle_ms;
    double iae, itae;
    uint32_t n;
};

static void loop_stats(const float *t_ms, const float *v, const float *d,
                       int n, float vref, LoopStats &s, float settle_band) {
    s.n = n;
    double t_end = t_ms[n - 1];
    s.rate_khz = (n - 1) / (t_end * 1.0) / 1e3;         // t in ms -> kHz
    s.ts_meas_us = t_end * 1000.0 / (n - 1);
    int sw = n - (n / 5);                                 // settled window
    int wn = n - sw;
    double m_sum = 0, d_sum = 0;
    s.ss_min = 1e9f; s.ss_max = -1e9f; s.d_min_s = 1e9f; s.d_max_s = -1e9f;
    for (int i = sw; i < n; i++) {
        m_sum += v[i]; d_sum += d[i];
        if (v[i] < s.ss_min) s.ss_min = v[i];
        if (v[i] > s.ss_max) s.ss_max = v[i];
        if (d[i] < s.d_min_s) s.d_min_s = d[i];
        if (d[i] > s.d_max_s) s.d_max_s = d[i];
    }
    s.ss_mean = m_sum / wn; s.d_mean = d_sum / wn;
    s.ss_err = s.ss_mean - vref;
    s.settle_ms = -1;
    for (int i = n - 1; i >= 0; i--)
        if (v[i] < vref - settle_band || v[i] > vref + settle_band) { s.settle_ms = t_ms[i]; break; }
    s.iae = 0; s.itae = 0;
    for (int i = 1; i < n; i++) {
        double e = v[i] - vref, ep = v[i - 1] - vref;
        double dt = (t_ms[i] - t_ms[i - 1]) * 1e-3;
        double ea = 0.5 * (fabs(e) + fabs(ep)) * dt;
        s.iae += ea; s.itae += ea * (t_ms[i] * 1e-3);
    }
}

// run one closed loop over the jumper at a target schedule (avg_n reads per
// sample) with a fixed set of gain-recipes; vectors passed by caller.

// ---------------- Mode L: latency breakdown ----------------
static void mode_latency() {
    const int N = 5000;
    auto tmin = [&](uint32_t a, uint32_t b) { return (b > a) ? (b - a) : 1; };
    uint32_t mn = 0xFFFFFFFF, mx = 0; uint64_t sm = 0;
    for (int i = 0; i < N; i++) { uint32_t a = cc(); (void)read_vpin(); uint32_t dt = tmin(a, cc()); if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt; }
    Serial.printf("CSV,L,adc_mv,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0 / MHZ / N, ns_span(mx));
    mn = 0xFFFFFFFF; mx = 0; sm = 0;
    for (int i = 0; i < N; i++) { uint32_t a = cc(); (void)read_vpin_raw(); uint32_t dt = tmin(a, cc()); if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt; }
    Serial.printf("CSV,L,adc_raw,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0 / MHZ / N, ns_span(mx));
    PIDF pid; pid.kp = KP; pid.ki = KI; pid.kd = KD; pid.ts = 6.2e-5f;
    mn = 0xFFFFFFFF; mx = 0; sm = 0;
    for (int i = 0; i < N; i++) { uint32_t a = cc(); float mq = fqabs(-12.0f + (i & 63) * 0.01f, 8); pid.step(mq, VREF); (void)fqduty(pid.d, 8); uint32_t dt = tmin(a, cc()); if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt; }
    Serial.printf("CSV,L,comp_f32,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0 / MHZ / N, ns_span(mx));
    PIDQ pq; pq.kp24 = (int32_t)(KP * (1 << 24)); pq.kits24 = (int32_t)(KI * 6.2e-5f * (1 << 24)); pq.kdts24 = (int32_t)((KD / 6.2e-5f) * (1 << 24));
    mn = 0xFFFFFFFF; mx = 0; sm = 0;
    for (int i = 0; i < N; i++) { uint32_t a = cc(); int32_t mq = q1q10(to_q10(-12.0f + (i & 63) * 0.01f), ADC_FS / 256.0f); pq.step(mq - to_q10(VREF)); (void)q1duty(pq.d_q15, 256); uint32_t dt = tmin(a, cc()); if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt; }
    Serial.printf("CSV,L,comp_i32,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0f / MHZ / N, ns_span(mx));
    // pure float PID step (no quantizer powf) -- isolates controller math
    PIDF pp; pp.kp = KP; pp.ki = KI; pp.kd = KD; pp.ts = 6.2e-5f;
    mn = 0xFFFFFFFF; mx = 0; sm = 0;
    for (int i = 0; i < N; i++) { uint32_t a = cc(); pp.step(-12.0f + (i & 63) * 0.01f, VREF); uint32_t dt = tmin(a, cc()); if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt; }
    Serial.printf("CSV,L,comp_f32_pure,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0f / MHZ / N, ns_span(mx));
    mn = 0xFFFFFFFF; mx = 0; sm = 0;
    pwm_init(RES_BITS);
    pwm_set(0.5f);
    for (int i = 0; i < N; i++) { uint32_t a = cc(); pwm_set(0.25f + (i & 1) * 0.5f); uint32_t dt = tmin(a, cc()); if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt; }
    Serial.printf("CSV,L,pwm,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0f / MHZ / N, ns_span(mx));
    // fast paths (bare-register on STM32, aliases on ESP32)
    mn = 0xFFFFFFFF; mx = 0; sm = 0;
    for (int i = 0; i < N; i++) { uint32_t a = cc(); (void)read_vpin_fast(); uint32_t dt = tmin(a, cc()); if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt; }
    Serial.printf("CSV,L,adc_fast,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0f / MHZ / N, ns_span(mx));
    mn = 0xFFFFFFFF; mx = 0; sm = 0;
    for (int i = 0; i < N; i++) { uint32_t a = cc(); pwm_set_fast(0.25f + (i & 1) * 0.5f); uint32_t dt = tmin(a, cc()); if (dt < mn) mn = dt; if (dt > mx) mx = dt; sm += dt; }
    Serial.printf("CSV,L,pwm_fast,%.2f,%.2f,%.2f\n", ns_span(mn), sm * 1000.0f / MHZ / N, ns_span(mx));
    pwm_set(0.5f);
    Serial.println("CSV,LDONE");
}

// ---------------- Mode R: effective resolution grid ----------------
static void mode_effres() {
    // sweep effective ADC (12/10/8/6) and effective PWM (8/6/4) resolutions
    // using the 8-bit LEDC wrapper + digital reduction, in the 8-bit sense.
    // Real PWM resolution at 100 kHz is capped at 8-bit (Mode H); effective
    // resolution below that is produced by re-quantizing the *commanded* duty
    // (duty rounding) which the LEDC then writes at its native 8-bit wing.
    const int N = 4000;
    const int LD = (N + LOGN - 1) / LOGN;   // decimate log to fit RAM
    extern float g_t[], g_v[], g_d[], g_st[];
    float *t = g_t, *v = g_v, *d = g_d;
    const int AVG = 64;                    // low-noise schedule (~6.8 ms)
    // sweep ADC 12/10/8/6 at PWM=8, then PWM 6/4 at ADC=8 (6 cells)
    const int ua[6] = {12, 10, 8, 6, 8, 8};
    const int up[6] = {8, 8, 8, 8, 6, 4};
    for (int c = 0; c < 6; c++) {
                        int ab = ua[c], pb = up[c];
            pwm_init(RES_BITS);
            // measure achieved period first (ADC-bound, independent of target)
            uint32_t t2 = micros();
            for (int m2 = 0; m2 < 8; m2++) (void)vout_of(read_vpin_avg(AVG));
            float ts_meas = (float)(micros() - t2) / 8.0f;   // us
            uint64_t t0 = micros();
            float filt = 0; bool fi = false;
            PIDF pf; pf.kp = 0.02f; pf.kd = 0.0f;
            pf.ki = 0.0034f / (ts_meas * 1e-6f);   // keep ki*ts = 3.4e-3
            pf.ts = ts_meas * 1e-6f;
            for (int i = 0; i < N; i++) {
                float mr = vout_of(read_vpin_avg(AVG));
                float m = fi ? 0.08f * mr + 0.92f * filt : mr; filt = m; fi = true;
                float mq = fqabs(m, ab);
                pf.step(mq, VREF);
                float dq = fqduty(pf.d, pb);
                pwm_set(dq);
                if (i % LD == 0) {
                    t[i / LD] = (float)(micros() - t0) / 1e3f; v[i / LD] = m; d[i / LD] = dq;
                }
            }
            LoopStats s; loop_stats(t, v, d, (N + LD - 1) / LD, VREF, s, 0.5f);
            float ripple_mv = (s.ss_max - s.ss_min) * 1000.0f;
            float d_ripple = s.d_max_s - s.d_min_s;
            Serial.printf("CSV,R,%d,%d,%.1f,%.1f,%.3f,%.1f,%.3f,%.3f,%.3f\n",
                ab, pb, s.ts_meas_us, s.ss_err * 1000.0f, ripple_mv, s.settle_ms,
                s.d_mean, d_ripple, s.iae);
    }
    pwm_set(0.5f);
    Serial.println("CSV,RDONE");
}

// ---------------- Mode S: schedule sweep (achievable periods) ----------------
static void mode_schedule() {
    // sweep avg_n = {1,8,32,64}: these map to ADC-bound achievable periods
    // ~62 us / ~0.5 ms / ~2 ms / ~6.8 ms. Both frozen and retuned gains.
    extern float g_t[], g_v[], g_d[], g_st[];
    float *t = g_t, *v = g_v, *d = g_d;
    for (int avg : {1, 8, 32, 64}) {
        for (int ret : {0, 1}) {
            float kp = 0.05f, ki = 100.0f, kd = 1e-5f;
            pwm_init(RES_BITS);
            // measure achieved period for THIS avg_n first
            uint32_t t2 = micros();
            for (int m2 = 0; m2 < 8; m2++) (void)vout_of(read_vpin_avg(avg));
            float ts_meas = (float)(micros() - t2) / 8.0f;   // us
            uint64_t t0 = micros();
            float filt = 0; bool fi = false;
            PIDF pf; pf.kp = kp; pf.ki = ki; pf.kd = kd; pf.ts = ts_meas * 1e-6f;
            if (ret) { pf.kp = 0.02f; pf.ki = 0.0034f / (ts_meas * 1e-6f); pf.kd = 0.0f; }
            int n = avg <= 8 ? 3000 : 1200;
            const int LD = (n + LOGN - 1) / LOGN;   // decimate log to fit RAM
            for (int i = 0; i < n; i++) {
                float mr = vout_of(read_vpin_avg(avg));
                float m = fi ? 0.08f * mr + 0.92f * filt : mr; filt = m; fi = true;
                pf.step(fqabs(m, 8), VREF);
                float dq = fqduty(pf.d, 8);
                pwm_set(dq);
                if (i % LD == 0) {
                    t[i / LD] = (float)(micros() - t0) / 1e3f; v[i / LD] = m; d[i / LD] = dq;
                }
            }
            LoopStats s; loop_stats(t, v, d, (n + LD - 1) / LD, VREF, s, 0.5f);
            float ripple_mv = (s.ss_max - s.ss_min) * 1000.0f;
            float d_ripple = s.d_max_s - s.d_min_s;
            Serial.printf("CSV,S,%d,%d,%.1f,%.3f,%.1f,%.3f,%.1f,%.4f,%.3f,%.3f\n",
                avg, ret, s.ts_meas_us, s.rate_khz, s.settle_ms,
                s.ss_err, ripple_mv, d_ripple, s.d_mean, s.iae);
        }
    }
    pwm_set(0.5f);
    Serial.println("CSV,SDONE");
}

// ---------------- Mode X: injected computational latency ----------------
static void mode_latency_inject() {
    // raw schedule (avg_n=1) with an injected busy-wait delay per sample.
    const int N = 2000;
    const int LD = (N + LOGN - 1) / LOGN;   // decimate log to fit RAM
    extern float g_t[], g_v[], g_d[], g_st[];
    float *t = g_t, *v = g_v, *d = g_d;
    const float dlys_us[] = {0.0f, 20.0f, 60.0f, 200.0f};
    for (float dl : dlys_us) {
        pwm_init(RES_BITS);
        uint32_t t2 = micros();
        for (int m2 = 0; m2 < 8; m2++) (void)vout_of(read_vpin_avg(1));
        float ts_meas = (float)(micros() - t2) / 8.0f;
        uint64_t t0 = micros();
        float filt = 0; bool fi = false;
        PIDF pf; pf.kp = 0.02f; pf.kd = 0.0f;
        pf.ki = 0.0034f / (ts_meas * 1e-6f);   // bandwidth-matched
        pf.ts = ts_meas * 1e-6f;
        for (int i = 0; i < N; i++) {
            float mr = vout_of(read_vpin_avg(1));
            float m = fi ? 0.08f * mr + 0.92f * filt : mr; filt = m; fi = true;
            pf.step(fqabs(m, 8), VREF);
            pwm_set(fqduty(pf.d, 8));
            if (dl > 0.0f) delayMicroseconds((uint32_t)dl);
            if (i % LD == 0) {
                t[i / LD] = (float)(micros() - t0) / 1e3f; v[i / LD] = m; d[i / LD] = pf.d;
            }
        }
        LoopStats s; loop_stats(t, v, d, (N + LD - 1) / LD, VREF, s, 0.5f);
        float ripple_mv = (s.ss_max - s.ss_min) * 1000.0f;
        Serial.printf("CSV,X,%.1f,%.1f,%.1f,%.3f,%.1f,%.3f\n",
            dl, s.ts_meas_us, s.settle_ms, s.ss_err, ripple_mv, s.iae);
    }
    pwm_set(0.5f);
    Serial.println("CSV,XDONE");
}

// ---------------- Mode A: adaptive schedule + dithering ----------------
// Quantization-activity indicator QAI = EMA of |e| (output-frame volts).
// Hysteresis: |e| > EH  -> fast schedule (avg_n=1); |e| < EL and QAI < QL
// -> slow schedule (avg_n=32); otherwise hold. Dithering activates when
// |e| < EL but QAI > QH (quantization-induced oscillation) to break the
// limit cycle. Update period resets per selected schedule.
static void mode_adaptive() {
    extern float g_t[], g_v[], g_d[], g_st[];
    float *t = g_t, *v = g_v, *d = g_d;
    int *st = (int *)g_st;       // 0=fast, 1=slow, 2=dithering
    int n = 0;
    pwm_init(RES_BITS);          // LEDC must be configured before pwm_set
    // Thresholds sit between the two noise floors:
    //   fast N=8  -> ~0.9 V rms post-EMA |e|
    //   slow N=64 -> ~0.2 V rms post-EMA |e|
    const float EH = 1.5f, EL = 0.75f, QH = 0.45f, QL = 0.25f;
    const int DEBOUNCE = 8;      // consecutive samples must agree to switch
    uint64_t t0 = micros();
    float filt = 0; bool fi = false;
    PIDF pf; pf.kp = 0.02f; pf.kd = 0.0f;
    float qai = 0.0f; bool fast = true; bool dither = false;
    int db = 0;
    float settle_obs = 12.0f;         // heavily smoothed |e| for mode decision
    const float SO_A = 0.02f;         // long time constant (~50 fast samples)
    float dither_phase = 0.5f;
    float vref = VREF;
    const int LD = 8;                 // decimate log to fit F103 RAM
    while (n < LOGN * LD && (micros() - t0) < 6e6) {
        float wall = (float)(micros() - t0) / 1e3f;
        if (wall > 2500.0f && wall < 2600.0f) vref = -6.0f;  // set-point step
        int avg = fast ? 8 : 64;
        pf.ki = fast ? 3.94f : 0.497f;   // ki*ts = 3.4e-3 constant (time-step-aware)
        pf.ts = (fast ? 863e-6f : 6.84e-3f);
        float mr = vout_of(read_vpin_avg(avg));
        float m = fi ? 0.08f * mr + 0.92f * filt : mr; filt = m; fi = true;
        float e = m - vref;
        // decision variables: QAI (fast EMA) for dithering, settle_obs
        // (slow EMA) for schedule switching on the MEAN error, immune to the
        // per-sample noise floor.
        qai = 0.5f * fabsf(e) + 0.5f * qai;
        settle_obs = SO_A * fabsf(e) + (1 - SO_A) * settle_obs;
        // schedule selection with debounce (time-hysteresis).
        // Exit fast only on sustained mean settle; re-enter only on large error.
        bool want_fast = fabsf(e) > EH;
        bool want_slow = !want_fast && settle_obs < EL;
        if (!fast && want_fast) { db++; if (db >= DEBOUNCE) { fast = true; db = 0; } }
        else if (fast && want_slow) { db++; if (db >= DEBOUNCE) { fast = false; db = 0; dither = false; } }
        else db = 0;
        // quantization mitigation: dither only in slow mode when QAI is
        // elevated (limit-cycle hint) but the mean error has settled.
        if (fast) dither = false;
        else dither = (fabsf(e) < EL && qai > QH) || (dither && qai > QL);
        float mq = fqabs(m, 8);
        pf.step(mq, vref);
        float dq = fqduty(pf.d, 8);
        if (dither) {
            dither_phase = -dither_phase;
            dq = fqduty(pf.d + dither_phase * 0.004f, 8);   // +-1 LSB (8-bit)
        }
        pwm_set(dq);
        if (n % LD == 0) {
            int j = n / LD;
            t[j] = wall; v[j] = m; d[j] = dq;
            st[j] = (dither ? 2 : (fast ? 0 : 1));
        }
        n++;
    }
    pwm_set(0.5f);
    int m = (n + LD - 1) / LD;
    for (int i = 0; i < m; i++)
        Serial.printf("CSV,A,%.3f,%.4f,%.4f,%d\n", t[i], v[i], d[i], (int)st[i]);
    Serial.println("CSV,ADONE");
}

// ---------------- Mode T: sampling-boundary test ----------------
// Paces the loop at exact Ts/Tsw multiples (1..5x) with the paper's frozen
// gains. The ESP32 can only achieve Ts >= ~73 us (7x Tsw), so rungs below
// that report ts_ach > ts_req = "not achievable". The F103 (~10 us raw loop)
// reaches every rung -- the hardware replication of the sim's 3.5-4x Tsw
// boundary. Metrics are streaming (settle/ss/ripple/IAE), no time-series log,
// so RAM cost is O(1).
static void mode_boundary() {
    const int N = 6000;                 // 60 ms per rung at 1x Tsw
    const int SENSE_BITS = 8, DUTY_BITS = 8;
    const float MULTS[] = {1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f, 5.0f};
    const int NM = 8;
    Serial.printf("CSV,TBEG,%d,%d,%d\n", N, SENSE_BITS, DUTY_BITS);
    for (int m = 0; m < NM; m++) {
        float mult = MULTS[m];
        uint32_t ts_us = (uint32_t)(TSW_US * mult);   // 10..50 us, exact
        pwm_init(RES_BITS);
        // int32 fixed-point pipeline: the float path (soft-float + powf in
        // the quantizer) costs ~264 us/step on the F103 and would make the
        // 1x-2x rungs unreachable; the fixed path is ~1.3 us.
        float ts_s = TSW_US * mult * 1e-6f;
        PIDQ pq; pq.kp24 = (int32_t)(KP * (1 << 24));
        pq.kits24 = (int32_t)(KI * ts_s * (1 << 24));
        pq.kdts24 = (int32_t)((KD / ts_s) * (1 << 24));
        const float stepA = ADC_FS / 256.0f;   // volts per 8-bit ADC code
        uint64_t t0 = micros();
        uint32_t next = ts_us;
        float iae = 0.0f;
        double vsum = 0; int vcnt = 0;
        float vmin = 1e9f, vmax = -1e9f;
        uint32_t last_out_i = 0;
        const int i0 = N - N / 5;       // final-20% window for ss/ripple
        const float BAND = 0.5f;        // settle band, matches other modes
        for (int i = 0; i < N; i++) {
            while ((uint32_t)(micros() - (uint32_t)t0) < next) { }
            float mr = vout_of(read_vpin_fast());
            int32_t mq = q1q10(to_q10(mr), stepA);
            pq.step(mq - to_q10(VREF));
            pwm_set_fast((float)q1duty(pq.d_q15, 256) / D_Q);
            float e = mr - VREF;
            iae += fabsf(e) * (TSW_US * mult * 1e-6f);
            if (fabsf(e) > BAND) last_out_i = i;
            if (i >= i0) {
                vsum += mr; vcnt++;
                if (mr < vmin) vmin = mr;
                if (mr > vmax) vmax = mr;
            }
            next += ts_us;
        }
        uint32_t t_end = micros();
        float ts_ach = (float)(t_end - t0) / N;
        bool achievable = ts_ach < 1.15f * (float)ts_us;
        float ss_err = (vcnt ? (float)(vsum / vcnt) : 0.0f) - VREF;
        float ripple_mv = (vmax - vmin) * 1000.0f;
        float settle_ms = last_out_i * (ts_ach * 1e-3f);
        Serial.printf("CSV,T,%.1f,%lu,%.1f,%d,%.3f,%.1f,%.3f\n",
                      mult, (unsigned long)ts_us, ts_ach, (int)achievable,
                      settle_ms, ss_err, ripple_mv);
        Serial.printf("CSV,T2,%.1f,%.4f\n", mult, iae);
    }
    pwm_set(0.5f);
    Serial.println("CSV,TDONE");
}

void setup() {
#ifdef ARDUINO_ARCH_ESP32
    Serial.begin(460800);
#else
    Serial.begin(115200);
#endif
    delay(300);
#ifdef ARDUINO_ARCH_ESP32
    MHZ = (float)getCpuFrequencyMhz();
    adcAttachPin(ADC_PIN);
    analogSetAttenuation(ADC_11db);
#else
    MHZ = (float)F_CPU / 1e6f;
    dwt_init();
    analogReadResolution(12);
    adc_fast_init();
#endif
    Serial.printf("CSV,BOOT,%s\n", PLATFORM_NAME);
    Serial.printf("CSV,CPU,%d\n", (int)MHZ);
#ifndef ARDUINO_ARCH_ESP32
    // implied-clock sanity: catches HSI fallback boots (clone crystals)
    uint32_t ca = cc(); delay(100); uint32_t cd = cc() - ca;
    Serial.printf("CSV,CLK,%lu MHz implied\n", (unsigned long)(cd / 100000UL));
#endif
}

void loop() {
    if (Serial.available()) {
        int c = Serial.read();
        if (c == 'C' || c == 'c') {
#ifdef ARDUINO_ARCH_ESP32
            // optional averaging count follows as digits (default 1)
            int acc = 0;
            while (Serial.available()) {
                int d = Serial.peek();
                if (d >= '0' && d <= '9') { acc = acc * 10 + (d - '0'); Serial.read(); }
                else break;
            }
            int avg_n = (acc > 0) ? constrain(acc, 1, 64) : 1;
            while (Serial.available()) Serial.read();   // drain rest
            Serial.printf("CSV,CAVG,%d\n", avg_n);
            for (int m : {1, 2, 5}) {
                mode_closed(m, avg_n, false);
            }
            Serial.printf("CSV,CRETUNE\n");
            for (int m : {1, 2, 5}) {
                mode_closed(m, avg_n, true);
            }
            Serial.println("CSV,CDONE");
#endif
            return;
        }
        while (Serial.available()) Serial.read();  // drain
        if (c == 'B') mode_bench();
        else if (c == 'H') mode_hw();
        else if (c == 'L' || c == 'l') mode_latency();
        else if (c == 'R' || c == 'r') mode_effres();
        else if (c == 'S' || c == 's') mode_schedule();
        else if (c == 'X' || c == 'x') mode_latency_inject();
        else if (c == 'A' || c == 'a') mode_adaptive();
        else if (c == 'T' || c == 't') mode_boundary();
    }
    delay(10);
}