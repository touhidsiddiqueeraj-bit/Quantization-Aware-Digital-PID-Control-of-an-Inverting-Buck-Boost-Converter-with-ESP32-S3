// Blue Pill F103C8 bring-up gate: flash + UART + DWT timing + ADC read.
// Gate for the full cross-platform port. Blink PC13 (active-low), print a
// banner, verify DWT cycle counter against a known delay, time analogRead.
#include <Arduino.h>

#define LED_PIN PC13
#define ADC_PIN PA1   // ADC1_IN1, matches the LC-plant sense pin plan

static inline uint32_t cc() { return dwt_getCycles(); }

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    dwt_init();  // core API: returns nonzero on success
    delay(300);
    Serial.println("CSV,BOOT,bluepill_f103c8");
    Serial.printf("CSV,CPU,%ld MHz\n", F_CPU / 1000000L);

    // DWT sanity: a 100 ms delay should be ~72e6 cycles * 0.1 = 7.2e6
    uint32_t a = cc();
    delay(100);
    uint32_t d = cc() - a;
    Serial.printf("CSV,DWT,delay100ms,%lu cycles, %lu MHz implied\n",
                  (unsigned long)d, (unsigned long)(d / 100000UL));

    // ADC read timing (single raw conversion)
    analogReadResolution(12);
    uint32_t mn = 0xFFFFFFFF, mx = 0; uint64_t sm = 0;
    const int N = 2000;
    for (int i = 0; i < N; i++) {
        uint32_t t0 = cc();
        (void)analogRead(ADC_PIN);
        uint32_t dt = cc() - t0;
        if (dt < mn) mn = dt;
        if (dt > mx) mx = dt;
        sm += dt;
    }
    Serial.printf("CSV,ADC,min %lu ns avg %lu ns max %lu ns\n",
                  (unsigned long)((uint64_t)mn * 1000000000ULL / F_CPU),
                  (unsigned long)((uint64_t)sm * 1000000000ULL / N / F_CPU),
                  (unsigned long)((uint64_t)mx * 1000000000ULL / F_CPU));

    // PWM sanity: 100 kHz on PA8 (TIM1_CH1) at 50%
    pinMode(PA8, OUTPUT);
    analogWriteFrequency(100000);
    analogWriteResolution(8);
    analogWrite(PA8, 128);
    Serial.printf("CSV,PWM,PA8 @100kHz 50pct set\n");
    Serial.println("CSV,BRINGUP,DONE");
}

void loop() {
    digitalWrite(LED_PIN, LOW);   // LED on (active low)
    delay(200);
    digitalWrite(LED_PIN, HIGH);  // LED off
    delay(800);
    static uint32_t beat = 0;
    Serial.printf("CSV,BEAT,%lu,adc %lu\n", (unsigned long)++beat,
                  (unsigned long)analogRead(ADC_PIN));
}
