#pragma once
// ECG R-Peak Detector — configuration
// Streaming (sample-by-sample) implementation.

// ---------------------------------------------------------------- sampling
#define ECG_PIN            A0
#define ADC_MIDPOINT      512      // 10-bit ADC centred at mid-rail (~1.65 V)

// Nominal sampling period. 2778 us ~ 360 Hz (MIT-BIH); 4000 us ~ 250 Hz.
// This is now a true period (micros() scheduler), not a post-read delay:
// the ~112 us spent inside analogRead() is accounted for, so the effective
// rate matches the nominal one.
#define SAMPLE_PERIOD_US  2778

// ---------------------------------------------------------------- algorithm
// Hermite smoothing passes actually applied at run time.
#define SMOOTH_ITERATIONS    3

// Upper bound compiled in (sets the size of the cascade state array).
// Each extra pass costs 5 bytes of SRAM and adds one sample of latency.
#define SMOOTH_MAX_ITERATIONS 8

// Squared-derivative threshold, in squared ADC counts.
// The derivative is NOT normalised by the sampling interval, so this value
// is device-specific; ~500 works for a 10-bit ADC centred at 512.
#define THRESHOLD          500

// Retrospective search window for the R apex, in samples, counted back from
// the derivative crossing. 30 samples ~ 83 ms at 360 Hz.
#define QRS_WIDTH           30

// Physiological refractory (blanking) period after an accepted R-peak.
#define REFRACTORY_MS      200

// ---------------------------------------------------------------- output
// 0 = MODE_FULL      : one CSV line per sample  (raw,smoothed,derivative)
// 1 = MODE_METADATA  : one line per detected R-peak only
// Changeable at run time with the M command (M0 / M1).
#define OUTPUT_MODE          0

// Output link: 0 = USB (Serial), 1 = Bluetooth (Serial1), 2 = both.
// Changeable at run time with the L command (L0 / L1 / L2).
#define OUTPUT_LINK          0

// Compile Bluetooth support in. Costs about 160 bytes of SRAM (the Serial1
// RX/TX ring buffers), so leave it at 0 for USB-only builds.
#define USE_BLUETOOTH        0

#define BAUD_RATE       115200    // USB; 250000 recommended for MODE_FULL
#define BT_BAUD_RATE    115200    // HC-05, must match the module configuration

// ---------------------------------------------------------------- lead-off
#define LO_PLUS_PIN         10
#define LO_MINUS_PIN        11
#define USE_LEAD_OFF         0    // set 1 if LO+/LO- are wired

// ---------------------------------------------------------------- derived
#define MODE_FULL            0
#define MODE_METADATA        1

#define LINK_USB             0
#define LINK_BT              1
#define LINK_BOTH            2

#if !USE_BLUETOOTH && (OUTPUT_LINK != LINK_USB)
#error "OUTPUT_LINK requires Bluetooth: set USE_BLUETOOTH to 1"
#endif

#define REFRACTORY_SAMPLES  ((uint32_t)((REFRACTORY_MS * 1000UL) / SAMPLE_PERIOD_US))

// Circular buffer must cover the retrospective window plus the pipeline delay
// (SMOOTH_MAX_ITERATIONS smoothing stages + 1 differentiation stage).
#define RAW_BUF_LEN         (QRS_WIDTH + SMOOTH_MAX_ITERATIONS + 4)
