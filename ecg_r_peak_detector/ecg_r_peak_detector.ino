// ECG R-Peak Detector — Arduino Mega 2560 + AD8232
// Algorithm: Hermite polynomial smoothing + threshold R-peak detection
// DOI: 10.32620/reks.2026.1.02

#include "config.h"

static int32_t g_threshold  = THRESHOLD;
static uint8_t g_iterations = SMOOTH_ITERATIONS;

static int16_t rawBuf[BUFFER_SIZE];
static int16_t smoothBuf[BUFFER_SIZE];
static int32_t derivSqBuf[BUFFER_SIZE];

#define MAX_PEAKS  ((BUFFER_SIZE / QRS_WIDTH) + 2)
static int16_t  peakIndex[MAX_PEAKS];
static int16_t  peakAmplitude[MAX_PEAKS];
static uint8_t  peakCount;
static uint32_t bufferNumber = 0;

// H2(ti) = (F[i-1] + 6*F[i] + F[i+1]) / 8  — one Hermite smoothing pass
static void smoothOnce(int16_t* buf, uint16_t len)
{
    if (len < 3) return;
    int16_t prev = buf[0];
    for (uint16_t i = 1; i < len - 1; i++) {
        int16_t curr = buf[i];
        buf[i] = (int16_t)(((int32_t)prev + 6*(int32_t)curr + (int32_t)buf[i+1] + 4) >> 3);
        prev = curr;
    }
}

static void smoothSignal(const int16_t* src, int16_t* dst, uint16_t len, uint8_t iters)
{
    for (uint16_t i = 0; i < len; i++) dst[i] = src[i];
    for (uint8_t it = 0; it < iters; it++) smoothOnce(dst, len);
}

// Squared central difference, zero on non-decreasing slopes
static void computeDerivSquared(const int16_t* smooth, int32_t* out, uint16_t len)
{
    out[0] = out[len-1] = 0;
    for (uint16_t i = 1; i < len - 1; i++) {
        int16_t d = smooth[i-1] - smooth[i+1];
        out[i] = (d > 0) ? (int32_t)d * d : 0;
    }
}

// Scan above-threshold intervals, find derivative peak, back-search raw max
static void findPeaks(uint16_t len)
{
    peakCount = 0;
    uint16_t i = 1;
    while (i < len && peakCount < MAX_PEAKS) {
        if (derivSqBuf[i] < g_threshold) { i++; continue; }

        uint16_t start = i, end = i;
        while (end < len && (end-start) < (uint16_t)QRS_WIDTH && derivSqBuf[end] >= g_threshold)
            end++;

        int32_t maxDer = derivSqBuf[start];
        uint16_t peakDerIdx = start;
        for (uint16_t j = start+1; j < end; j++)
            if (derivSqBuf[j] > maxDer) { maxDer = derivSqBuf[j]; peakDerIdx = j; }

        uint16_t searchStart = (peakDerIdx >= (uint16_t)QRS_WIDTH) ? peakDerIdx - (uint16_t)QRS_WIDTH : 0;
        int16_t maxAmp = rawBuf[searchStart];
        uint16_t maxIdx = searchStart;
        for (uint16_t j = searchStart+1; j < peakDerIdx+1; j++)
            if (rawBuf[j] > maxAmp) { maxAmp = rawBuf[j]; maxIdx = j; }

        peakIndex[peakCount]     = (int16_t)maxIdx;
        peakAmplitude[peakCount] = maxAmp;
        peakCount++;
        i = end + 1;
    }
}

static void sendOutput()
{
    Serial.print(F("BUF:")); Serial.println(bufferNumber);

    Serial.print(F("RAW:"));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        Serial.print(rawBuf[i]);
        if (i < BUFFER_SIZE-1) Serial.print(',');
    }
    Serial.println();

    Serial.print(F("SMO:"));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        Serial.print(smoothBuf[i]);
        if (i < BUFFER_SIZE-1) Serial.print(',');
    }
    Serial.println();

    Serial.print(F("DER:"));
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        Serial.print(derivSqBuf[i]);
        if (i < BUFFER_SIZE-1) Serial.print(',');
    }
    Serial.println();

    Serial.print(F("THR:")); Serial.println(g_threshold);

    // RPK: index only; host computes RR = (idx2-idx1) * Dt
    Serial.print(F("RPK:"));
    if (peakCount == 0) {
        Serial.println(F("NONE"));
    } else {
        for (uint8_t p = 0; p < peakCount; p++) {
            Serial.print(peakIndex[p]);
            if (p < peakCount-1) Serial.print(',');
        }
        Serial.println();
    }
    Serial.println(F("END"));
}

// T<n> → threshold,  I<n> → smooth iterations
static void handleSerial()
{
    if (!Serial.available()) return;
    char cmd = Serial.read();
    if (cmd == 'T' || cmd == 'I') {
        long val = Serial.parseInt();
        if (cmd == 'T' && val > 0) {
            g_threshold = val;
            Serial.print(F("# threshold=")); Serial.println(g_threshold);
        } else if (cmd == 'I' && val >= 1 && val <= 200) {
            g_iterations = (uint8_t)val;
            Serial.print(F("# iterations=")); Serial.println(g_iterations);
        }
    }
}

static bool fillBuffer()
{
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
#if USE_LEAD_OFF
        if (digitalRead(LO_PLUS_PIN) || digitalRead(LO_MINUS_PIN)) {
            for (uint16_t j = i; j < BUFFER_SIZE; j++) rawBuf[j] = 0;
            return false;
        }
#endif
        rawBuf[i] = (int16_t)(analogRead(ECG_PIN) - 512); // centre 10-bit ADC
        delayMicroseconds(SAMPLE_DELAY_US);
    }
    return true;
}

void setup()
{
    Serial.begin(BAUD_RATE);
    while (!Serial && millis() < 3000) {}
#if USE_LEAD_OFF
    pinMode(LO_PLUS_PIN,  INPUT);
    pinMode(LO_MINUS_PIN, INPUT);
#endif
    Serial.println(F("# ECG R-Peak Detector (AD8232)"));
    Serial.print(F("# buffer="));     Serial.println(BUFFER_SIZE);
    Serial.print(F("# iterations=")); Serial.println(g_iterations);
    Serial.print(F("# threshold="));  Serial.println(g_threshold);
}

void loop()
{
    handleSerial();

    if (!fillBuffer()) {
        Serial.print(F("BUF:")); Serial.println(bufferNumber);
        Serial.println(F("LEADS_OFF"));
        Serial.println(F("END"));
        bufferNumber++;
        return;
    }

    smoothSignal(rawBuf, smoothBuf, BUFFER_SIZE, g_iterations);
    computeDerivSquared(smoothBuf, derivSqBuf, BUFFER_SIZE);
    findPeaks(BUFFER_SIZE);
    sendOutput();
    bufferNumber++;
}
