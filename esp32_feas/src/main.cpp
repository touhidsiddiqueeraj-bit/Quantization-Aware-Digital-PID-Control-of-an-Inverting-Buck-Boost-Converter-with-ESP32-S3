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
#include <esp_timer.h>

#define PWM_PIN   2
#define ADC_PIN   1         // ADC1_CH0
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
static inline uint32_t cc() { uint32_t c; asm volatile("rsr %0, ccount" : "=r"(c)); return c; }
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

// ---------------- hardware (Arduino LEDC) ----------------
static uint8_t g_res_bits = 8;
static uint8_t g_ch = 0;
static uint8_t g_pin = PWM_PIN;
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
        uint32_t t0 = esp_timer_get_time();
        for (int i = 0; i < 10; i++) { vout_of(read_vpin_avg(avg_n)); }
        uint32_t t1 = esp_timer_get_time();
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
        uint64_t t_start = esp_timer_get_time();
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
            log_t[done] = (float)(esp_timer_get_time() - t_start) / 1e3f; // ms
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

// ---------------- mode H: hardware characterization ----------------
static void mode_hw() {
    // LEDC achievable frequency per resolution at a 100 kHz request
    for (int bits = 1; bits <= 12; bits++) {
        uint32_t actual = ledcSetup(g_ch, (uint32_t)FSW, (uint8_t)bits);
        Serial.printf("CSV,H,%d,%u,%d\n", bits, actual, (actual ? 1 : 0));
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

void setup() {
    Serial.begin(460800);
    delay(300);
    MHZ = (float)getCpuFrequencyMhz();
    Serial.printf("CSV,BOOT\n");
    Serial.printf("CSV,CPU,%d\n", (int)MHZ);
    adcAttachPin(ADC_PIN);
    analogSetAttenuation(ADC_11db);
}

void loop() {
    if (Serial.available()) {
        int c = Serial.read();
        if (c == 'C' || c == 'c') {
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
            return;
        }
        while (Serial.available()) Serial.read();  // drain
        if (c == 'B') mode_bench();
        else if (c == 'H') mode_hw();
    }
    delay(10);
}