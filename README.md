# ECG R-Peak Detector

Real-time ECG acquisition and R-peak detection using Arduino Mega 2560 + AD8232.

## Hardware
- Arduino Mega 2560
- AD8232 ECG module
- Standard lead-II electrode placement

## Files
- `ecg_r_peak_detector/ecg_r_peak_detector.ino` — Arduino sketch
- `ecg_r_peak_detector/config.h` — configuration constants

## Configuration (config.h)

### `BUFFER_SIZE` (default: `128`)
Number of ADC samples collected before one frame is sent over Serial.
Larger values reduce Serial overhead but increase latency.
Safe range: 64–128 for Uno/Nano (limited RAM), 128–512 for Mega.

### `SMOOTH_ITERATIONS` (default: `3`)
Number of Hermite smoothing passes applied to the raw signal.
Each pass replaces every sample with a weighted average of itself and its neighbours:
`H[i] = (F[i-1] + 6·F[i] + F[i+1]) / 8`
2–3 passes are sufficient for a clean signal; increase up to 150 for a very noisy electrode contact.

### `THRESHOLD` (default: `500`)
Squared-derivative threshold for R-peak detection.
The algorithm computes `(smooth[i] - smooth[i-1])²` and marks a peak candidate when this value exceeds the threshold.
Tune this per device: for a 10-bit ADC centred at 512 (~1.65 V mid-rail), 500 works well.
Increase if false positives appear; decrease if peaks are missed.

### `QRS_WIDTH` (default: `30`)
Minimum number of samples that must separate two consecutive R-peaks.
Prevents double-detection within a single QRS complex.
At 360 Hz: 30 samples ≈ 83 ms (a typical QRS duration is 60–120 ms).

### `ECG_PIN` (default: `A0`)
Arduino analog input pin connected to the AD8232 OUTPUT pin.

### `LO_PLUS_PIN` / `LO_MINUS_PIN` (default: `10` / `11`)
Arduino digital input pins connected to AD8232 LO+ and LO− (lead-off detection outputs).
Only used when `USE_LEAD_OFF` is set to `1`.

### `USE_LEAD_OFF` (default: `0`)
Set to `1` to enable lead-off detection via LO+/LO− pins.
When leads are off, the frame is marked `LEADS_OFF` and skipped by the viewer.
Set to `0` if LO+/LO− are not wired — saves two digital reads per sample.

### `BAUD_RATE` (default: `115200`)
Serial communication speed. Must match the receiver (Python viewer or Serial Monitor).

### `SAMPLE_DELAY_US` (default: `2778`)
Delay in microseconds between ADC samples, controlling the effective sample rate.
`2778 µs → ~360 Hz` (matches MIT-BIH Arrhythmia Database standard).
`4000 µs → 250 Hz` (lower rate, less data).
Do not set below ~1000 µs — the AD8232 output bandwidth and Arduino ADC conversion time impose practical limits.

## Serial Protocol
Each buffer is sent as one frame:
```
BUF:<n>
RAW:<v0>,<v1>,...
SMO:<v0>,<v1>,...
DER:<v0>,<v1>,...
THR:<value>
RPK:<idx>:<amp>,...   (or RPK:NONE)
END
```
`RPK` lists detected R-peak positions as `index:amplitude` pairs, where `index` is the sample offset within the current buffer.

## License
MIT
