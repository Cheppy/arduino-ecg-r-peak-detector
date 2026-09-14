// ECG R-Peak Detector — Arduino Mega 2560 + AD8232
// Streaming (sample-by-sample) implementation.
//
// Pipeline, executed once per acquired sample:
//   raw -> K cascaded 3-tap Hermite smoothing stages
//       -> squared central-difference derivative (kept on falling slopes)
//       -> threshold + refractory blanking
//       -> retrospective search for the R apex in the raw circular buffer
//
// State is bounded and independent of recording length: the raw circular
// buffer is RAW_BUF_LEN samples, each smoothing stage holds two samples.
//
// Reference: https://doi.org/10.32620/reks.2026.1.02

#include <stdio.h>
#include "config.h"

// ----------------------------------------------------------------- state
static int32_t g_threshold  = THRESHOLD;
static uint8_t g_iterations = SMOOTH_ITERATIONS;
static uint8_t g_outputMode = OUTPUT_MODE;
static uint8_t g_link       = OUTPUT_LINK;

#if USE_BLUETOOTH
#define BT_PORT Serial1
#endif

// One 3-tap smoothing stage: y[n] = (x[n-1] + 6*x[n] + x[n+1] + 4) >> 3
// Emitting y[n] requires x[n+1], so every stage delays the stream by one sample.
typedef struct {
    int16_t x0;      // x[n-1]
    int16_t x1;      // x[n]
    uint8_t primed;  // 0,1 = filling; 2 = running
} Smooth3;

static Smooth3  g_stage[SMOOTH_MAX_ITERATIONS];

// Central-difference delay line over the smoothed stream.
static int16_t  g_s0, g_s1;
static uint8_t  g_dPrimed;

// Raw circular buffer, used only for the retrospective R-apex search.
static int16_t  g_raw[RAW_BUF_LEN];
static uint16_t g_rawPos;            // position of the most recent sample

static uint32_t g_sampleIndex;       // index of the most recent raw sample
static uint32_t g_lastPeakIndex;
static bool     g_havePeak;

// Above-threshold run currently being tracked.
static bool     g_inRun;
static uint32_t g_runStart;
static int32_t  g_runMaxDer;
static uint32_t g_runMaxIdx;
static int16_t  g_apexAmp;
static uint32_t g_apexIdx;

static uint32_t g_nextSampleMicros;
static uint32_t g_overruns;          // sampling instants missed
static uint32_t g_droppedLines;      // sample lines dropped, TX buffer full
static uint32_t g_peakCount;

// ----------------------------------------------------------------- output
// The pipeline is link-agnostic: the same byte stream goes to USB, to the
// Bluetooth module, or to both, depending on g_link.
static inline bool linkUsb() { return g_link == LINK_USB || g_link == LINK_BOTH; }
static inline bool linkBt()  { return g_link == LINK_BT  || g_link == LINK_BOTH; }

// Free space on the narrowest selected link.
static int txSpace()
{
    int space = 32767;
    if (linkUsb()) {
        int s = Serial.availableForWrite();
        if (s < space) space = s;
    }
#if USE_BLUETOOTH
    if (linkBt()) {
        int s = BT_PORT.availableForWrite();
        if (s < space) space = s;
    }
#endif
    return space;
}

static void txWrite(const char* s, int len)
{
    if (linkUsb()) Serial.write((const uint8_t*)s, (size_t)len);
#if USE_BLUETOOTH
    if (linkBt())  BT_PORT.write((const uint8_t*)s, (size_t)len);
#endif
}

// Sample lines are droppable: acquisition must never stall on a UART.
static void txSample(const char* s, int len)
{
    if (len <= 0) return;
    if (txSpace() < len) { g_droppedLines++; return; }
    txWrite(s, len);
}

// Peak lines are never dropped (a few dozen bytes per beat).
static void txPeak(const char* s, int len)
{
    if (len > 0) txWrite(s, len);
}

#if USE_BLUETOOTH
#define TX_BANNER(lit) do { if (linkUsb()) Serial.println(F(lit)); \
                            if (linkBt())  BT_PORT.println(F(lit)); } while (0)
#else
#define TX_BANNER(lit) do { if (linkUsb()) Serial.println(F(lit)); } while (0)
#endif

static void emitSample(int16_t raw, int16_t smooth, int32_t der)
{
    if (g_outputMode != MODE_FULL) return;
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "%d,%d,%ld\n", (int)raw, (int)smooth, (long)der);
    txSample(buf, n);
}

static void emitPeak(uint32_t idx, int16_t amp, uint32_t rrSamples)
{
    char buf[48];
    uint32_t rrMs = (rrSamples * (uint32_t)SAMPLE_PERIOD_US) / 1000UL;
    int n = snprintf(buf, sizeof(buf), "R,%lu,%d,%lu\n",
                     (unsigned long)idx, (int)amp, (unsigned long)rrMs);
    txPeak(buf, n);
}

static void emitStats()
{
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
                     "# n=%lu peaks=%lu overruns=%lu dropped=%lu mode=%u link=%u thr=%ld iter=%u\n",
                     (unsigned long)g_sampleIndex, (unsigned long)g_peakCount,
                     (unsigned long)g_overruns,    (unsigned long)g_droppedLines,
                     (unsigned)g_outputMode, (unsigned)g_link, (long)g_threshold,
                     (unsigned)g_iterations);
    txPeak(buf, n);
}

// ----------------------------------------------------------------- pipeline
static inline bool smooth3_push(Smooth3* s, int16_t xnew, int16_t* out)
{
    if (s->primed < 2) {
        if (s->primed == 0) s->x0 = xnew; else s->x1 = xnew;
        s->primed++;
        return false;
    }
    *out = (int16_t)(((int32_t)s->x0 + 6 * (int32_t)s->x1 + (int32_t)xnew + 4) >> 3);
    s->x0 = s->x1;
    s->x1 = xnew;
    return true;
}

static inline int16_t rawAt(uint32_t idx)
{
    uint16_t back = (uint16_t)(g_sampleIndex - idx);
    uint16_t p    = (uint16_t)((g_rawPos + RAW_BUF_LEN - back) % RAW_BUF_LEN);
    return g_raw[p];
}

// Track the above-threshold run of the squared derivative.
//
// The R apex is the maximum of the RAW signal over the retrospective window
// [runStart - QRS_WIDTH, runEnd]. The part of that window that precedes the run
// is scanned once, when the run opens; the rest is folded in incrementally as
// each new sample arrives, so no additional buffering is needed.
static void finalizeRun(void)
{
    uint32_t rr = g_havePeak ? (g_apexIdx - g_lastPeakIndex) : 0;
    g_lastPeakIndex = g_apexIdx;
    g_havePeak      = true;
    g_peakCount++;
    emitPeak(g_apexIdx, g_apexAmp, rr);
    g_inRun = false;
}

static void openRun(uint32_t m, int32_t der)
{
    g_inRun     = true;
    g_runStart  = m;
    g_runMaxDer = der;
    g_runMaxIdx = m;

    uint32_t start = (m > (uint32_t)QRS_WIDTH) ? (m - (uint32_t)QRS_WIDTH) : 0;
    g_apexAmp = rawAt(m);
    g_apexIdx = m;
    for (uint32_t j = start; j < m; j++) {
        int16_t r = rawAt(j);
        if (r > g_apexAmp) { g_apexAmp = r; g_apexIdx = j; }
    }
}

static void detectPeak(uint32_t m, int32_t der)
{
    if (g_havePeak && (m - g_lastPeakIndex) < REFRACTORY_SAMPLES) {
        g_inRun = false;
        return;
    }

    if (der >= g_threshold) {
        if (!g_inRun) {
            openRun(m, der);
        } else {
            if (der > g_runMaxDer) { g_runMaxDer = der; g_runMaxIdx = m; }
            int16_t r = rawAt(m);
            if (r > g_apexAmp) { g_apexAmp = r; g_apexIdx = m; }
        }
        // Bound the run so a sustained artefact cannot hold it open indefinitely.
        if ((m - g_runStart) >= (uint32_t)QRS_WIDTH) finalizeRun();
    } else if (g_inRun) {
        finalizeRun();
    }
}

static void processSample(int16_t raw)
{
    g_rawPos = (uint16_t)((g_rawPos + 1) % RAW_BUF_LEN);
    g_raw[g_rawPos] = raw;
    g_sampleIndex++;

    int16_t v = raw;
    for (uint8_t k = 0; k < g_iterations; k++) {
        if (!smooth3_push(&g_stage[k], v, &v)) return;  // cascade still priming
    }
    // v is the smoothed value at index g_sampleIndex - g_iterations

    if (g_dPrimed < 2) {
        if (g_dPrimed == 0) g_s0 = v; else g_s1 = v;
        g_dPrimed++;
        return;
    }

    int16_t d   = (int16_t)(g_s0 - v);              // > 0 on a falling slope
    int32_t der = (d > 0) ? (int32_t)d * (int32_t)d : 0;
    int16_t centre = g_s1;
    g_s0 = g_s1;
    g_s1 = v;

    uint32_t m = g_sampleIndex - (uint32_t)g_iterations - 1;
    emitSample(rawAt(m), centre, der);
    detectPeak(m, der);
}

static void resetPipeline()
{
    for (uint8_t k = 0; k < SMOOTH_MAX_ITERATIONS; k++) g_stage[k].primed = 0;
    for (uint16_t i = 0; i < RAW_BUF_LEN; i++) g_raw[i] = 0;
    g_dPrimed = 0;
    g_rawPos = 0;
    g_sampleIndex = 0;
    g_lastPeakIndex = 0;
    g_havePeak = false;
    g_inRun = false;
    g_runStart = 0;
    g_runMaxDer = 0;
    g_runMaxIdx = 0;
    g_apexAmp = 0;
    g_apexIdx = 0;
    g_peakCount = 0;
    g_overruns = 0;
    g_droppedLines = 0;
}

// ----------------------------------------------------------------- commands
// Commands arrive on whichever link is talking.
static Stream* cmdStream()
{
    if (Serial.available()) return &Serial;
#if USE_BLUETOOTH
    if (BT_PORT.available()) return &BT_PORT;
#endif
    return NULL;
}

// T<n> threshold | I<n> smoothing passes | M<0|1> output mode
// L<0|1|2> output link | S stats | X reset
static void handleSerial()
{
    Stream* in = cmdStream();
    if (in == NULL) return;
    char cmd = (char)in->read();
    switch (cmd) {
        case 'T': {
            long v = in->parseInt();
            if (v > 0) { g_threshold = v; emitStats(); }
            break;
        }
        case 'I': {
            long v = in->parseInt();
            if (v >= 1 && v <= SMOOTH_MAX_ITERATIONS) {
                g_iterations = (uint8_t)v;
                resetPipeline();
                emitStats();
            }
            break;
        }
        case 'M': {
            long v = in->parseInt();
            if (v == MODE_FULL || v == MODE_METADATA) {
                g_outputMode = (uint8_t)v;
                emitStats();
            }
            break;
        }
        case 'L': {
            long v = in->parseInt();
#if USE_BLUETOOTH
            if (v == LINK_USB || v == LINK_BT || v == LINK_BOTH) {
#else
            if (v == LINK_USB) {
#endif
                g_link = (uint8_t)v;
                emitStats();
            }
            break;
        }
        case 'S': emitStats(); break;
        case 'X': resetPipeline(); emitStats(); break;
        default: break;
    }
}

// ----------------------------------------------------------------- sketch
void setup()
{
    Serial.begin(BAUD_RATE);
    while (!Serial && millis() < 3000) {}
#if USE_BLUETOOTH
    BT_PORT.begin(BT_BAUD_RATE);
#endif
#if USE_LEAD_OFF
    pinMode(LO_PLUS_PIN,  INPUT);
    pinMode(LO_MINUS_PIN, INPUT);
#endif
    resetPipeline();
    TX_BANNER("# ECG R-Peak Detector (AD8232), streaming");
    {
        char b[64];
        int n = snprintf(b, sizeof(b), "# fs_nominal_hz=%lu refractory=%lu raw_buf=%u\n",
                         (unsigned long)(1000000UL / SAMPLE_PERIOD_US),
                         (unsigned long)REFRACTORY_SAMPLES, (unsigned)RAW_BUF_LEN);
        txPeak(b, n);
    }
    emitStats();
    TX_BANNER("# MODE_FULL lines: raw,smoothed,derivative_squared");
    TX_BANNER("# peak lines: R,sample_index,amplitude,rr_ms");
    g_nextSampleMicros = micros();
}

void loop()
{
    handleSerial();

    uint32_t now = micros();
    if ((int32_t)(now - g_nextSampleMicros) < 0) return;

    g_nextSampleMicros += (uint32_t)SAMPLE_PERIOD_US;
    // If output or a command stole more than one period, resynchronise and count it.
    if ((int32_t)(micros() - g_nextSampleMicros) > (int32_t)SAMPLE_PERIOD_US) {
        g_nextSampleMicros = micros() + (uint32_t)SAMPLE_PERIOD_US;
        g_overruns++;
    }

#if USE_LEAD_OFF
    if (digitalRead(LO_PLUS_PIN) || digitalRead(LO_MINUS_PIN)) {
        static uint32_t lastWarn = 0;
        if (millis() - lastWarn > 1000) { lastWarn = millis(); txPeak("# LEADS_OFF\n", 12); }
        resetPipeline();
        return;
    }
#endif

    processSample((int16_t)(analogRead(ECG_PIN) - ADC_MIDPOINT));
}
