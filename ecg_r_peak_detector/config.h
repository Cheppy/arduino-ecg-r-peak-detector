#pragma once
// ECG R-Peak Detector — configuration

// Buffer (Uno/Nano: 64-128, Mega: 256-512)
#define BUFFER_SIZE       128

// Hermite smoothing passes (2-3 typical, up to 150 for noisy signal)
#define SMOOTH_ITERATIONS   3

// Squared-derivative threshold (tune per device; ~500 for 10-bit ADC centred at 512)
#define THRESHOLD         500

// Max QRS width in samples (30 ≈ 83 ms at 360 Hz)
#define QRS_WIDTH          30

#define ECG_PIN            A0
#define LO_PLUS_PIN        10
#define LO_MINUS_PIN       11

// Set 0 if LO+/LO- pins are not wired
#define USE_LEAD_OFF       0

#define BAUD_RATE          115200

// 2778 µs ≈ 360 Hz (MIT-BIH standard); 4000 µs ≈ 250 Hz
#define SAMPLE_DELAY_US    2778
