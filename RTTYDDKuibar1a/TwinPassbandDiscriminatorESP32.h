#ifndef TWIN_PASSBAND_DISCRIMINATOR_ESP32_H
#define TWIN_PASSBAND_DISCRIMINATOR_ESP32_H

#include <Arduino.h>
#include <math.h>

// ============================================================
// Simple second-order resonant IIR filter
// ============================================================

class ResonantFilterESP32 {
private:
    float a0, a1;
    float b0, b1;

    float x1, x2;
    float y1, y2;

public:
    ResonantFilterESP32()
        : a0(0), a1(0), b0(0), b1(0),
          x1(0), x2(0), y1(0), y2(0) {}

    ResonantFilterESP32(float normalizedFreq, float r) {
        configure(normalizedFreq, r);
    }

    void configure(float normalizedFreq, float r) {
        float w = 2.0f * PI * normalizedFreq;

        a0 = -2.0f * r * cosf(w);
        a1 = r * r;

        b0 = (1.0f - a1) * 0.5f;
        b1 = -b0;

        x1 = x2 = 0.0f;
        y1 = y2 = 0.0f;
    }

    float filter(float input) {
        float result =
            b0 * input
            + b1 * x1
            - a0 * y1
            - a1 * y2;

        x2 = x1;
        x1 = input;

        y2 = y1;
        y1 = result;

        return result;
    }
};


// ============================================================
// Moving average
// ============================================================

class MovingAverageESP32 {
private:
    float *buffer;
    int length;
    int index;
    float sum;

public:
    MovingAverageESP32()
        : buffer(nullptr), length(0), index(0), sum(0) {}

    void begin(int n) {
        if (buffer) {
            delete[] buffer;
            buffer = nullptr;
        }

        length = max(1, n);
        buffer = new float[length];

        index = 0;
        sum = 0;

        for (int i = 0; i < length; i++)
            buffer[i] = 0.0f;
    }

    float run(float value) {
        sum -= buffer[index];
        sum += value;

        buffer[index] = value;

        index++;
        if (index >= length)
            index = 0;

        return sum / length;
    }

    float value() const {
        if (!buffer || length == 0)
            return 0.0f;

        return sum / length;
    }
};


// ============================================================
// Exponential smoother
// ============================================================

class SmootherESP32 {
private:
    float alpha;
    float state;

public:
    SmootherESP32()
        : alpha(1.0f), state(0.0f) {}

    void begin(float a, float seed = 0.0f) {
        alpha = a;
        state = seed;
    }

    float run(float input) {
        state = input * alpha + state * (1.0f - alpha);
        return state;
    }

    float value() const {
        return state;
    }
};


// ============================================================
// Schmitt trigger
// ============================================================

class HysteresisESP32 {
private:
    bool state;
    float lower;
    float upper;

public:
    HysteresisESP32()
        : state(false), lower(-0.1f), upper(0.1f) {}

    void begin(float low, float high) {
        lower = low;
        upper = high;
        state = false;
    }

    bool run(float input) {

        if (state) {
            if (input < lower)
                state = false;
        }
        else {
            if (input >= upper)
                state = true;
        }

        return state;
    }

    bool value() const {
        return state;
    }

    float high() const {
        return upper;
    }

    float low() const {
        return lower;
    }

    void high(float v) {
        upper = v;
    }

    void low(float v) {
        lower = v;
    }
};


// ============================================================
// Twin Passband Discriminator
// ============================================================

class TwinPassbandDiscriminatorESP32 {

public:

    struct Config {

        // ADC/audio sample rate
        float fs;

        // Center frequency
        float f0;

        // Difference between the two tones
        float shift;

        // Resonator coefficient.
        // Higher = narrower filter.
        float R;

        // RTTY baud rate
        float bps;

        // Envelope averaging length in bit-times
        float int_len;

        // Hysteresis threshold
        float trigger;

        Config()
            : fs(16000.0f),
              f0(1475.0f),
              shift(450.0f),
              R(0.97f),
              bps(50.0f),
              int_len(0.5f),
              trigger(0.10f)
        {}
    };


private:

    ResonantFilterESP32 high;
    ResonantFilterESP32 low;

    MovingAverageESP32 avgHigh;
    MovingAverageESP32 avgLow;

    SmootherESP32 bitLPF;

    HysteresisESP32 trigger;

    float sampleHigh;
    float sampleLow;

    float levelHigh;
    float levelLow;

    float angleValue;

    bool outputValue;


public:

    TwinPassbandDiscriminatorESP32(const Config &cfg) {

        // ----------------------------------------------------
        // Tone frequencies
        //
        // high = f0 + shift/2
        // low  = f0 - shift/2
        // ----------------------------------------------------

        float highFreq = cfg.f0 + cfg.shift * 0.5f;
        float lowFreq  = cfg.f0 - cfg.shift * 0.5f;

        high.configure(
            highFreq / cfg.fs,
            cfg.R
        );

        low.configure(
            lowFreq / cfg.fs,
            cfg.R
        );


        // ----------------------------------------------------
        // Original KK5JY used:
        //
        // int_len * fs / tone_frequency
        //
        // We retain that idea, but make the result safe.
        // ----------------------------------------------------

        int highAvgLength =
            max(1, (int)(cfg.int_len * cfg.fs / highFreq));

        int lowAvgLength =
            max(1, (int)(cfg.int_len * cfg.fs / lowFreq));

        avgHigh.begin(highAvgLength);
        avgLow.begin(lowAvgLength);


        // ----------------------------------------------------
        // Bit-stream low-pass filter
        //
        // Original:
        //
        // LowpassToAlpha(fs, bps * 2)
        // ----------------------------------------------------

        float fc = cfg.bps * 2.0f;

        float rc = 1.0f / (2.0f * PI * fc);
        float dt = 1.0f / cfg.fs;

        float alpha = dt / (rc + dt);

        bitLPF.begin(alpha);


        // ----------------------------------------------------
        // Schmitt trigger
        //
        // angle is in radians.
        //
        // PI/4 = 0.7854 rad
        //
        // trigger = 0.10 means about 7.85% of full
        // discriminator range.
        // ----------------------------------------------------

        float threshold =
            cfg.trigger * (PI / 4.0f);

        trigger.begin(
            -threshold,
            +threshold
        );


        sampleHigh = 0;
        sampleLow = 0;

        levelHigh = 0;
        levelLow = 0;

        angleValue = 0;

        outputValue = false;
    }


    // ========================================================
    // Process one ADC sample
    // ========================================================

    bool run(float sample) {

        // ----------------------------------------------------
        // Two resonant filters
        // ----------------------------------------------------

        sampleHigh = high.filter(sample);
        sampleLow  = low.filter(sample);


        // ----------------------------------------------------
        // Envelope magnitude
        // ----------------------------------------------------

        float absHigh = fabsf(sampleHigh);
        float absLow  = fabsf(sampleLow);


        // ----------------------------------------------------
        // Long-term average of each tone
        // ----------------------------------------------------

        levelHigh = avgHigh.run(absHigh);
        levelLow  = avgLow.run(absLow);


        // ----------------------------------------------------
        // Calculate discriminator angle
        //
        // Original:
        //
        // atan(hi / lo) - PI/4
        //
        // atan2() is safer when the low level becomes
        // extremely small.
        // ----------------------------------------------------

        if (levelLow < 0.000001f &&
            levelHigh < 0.000001f) {

            angleValue = 0.0f;

        }
        else {

            angleValue =
                atan2f(levelHigh, levelLow)
                - (PI / 4.0f);
        }


        // ----------------------------------------------------
        // LPF the bit waveform
        // ----------------------------------------------------

        angleValue = bitLPF.run(angleValue);


        // ----------------------------------------------------
        // Schmitt trigger
        //
        // IMPORTANT:
        //
        // There is deliberately NO trigger.offset()
        // here yet.
        //
        // We first want to establish that the basic
        // discriminator works correctly.
        // ----------------------------------------------------

        outputValue = trigger.run(angleValue);

        return outputValue;
    }


    // ========================================================
    // Accessors
    // ========================================================

    bool value() const {
        return outputValue;
    }

    float angle() const {
        return angleValue;
    }

    float levelHighValue() const {
        return levelHigh;
    }

    float levelLowValue() const {
        return levelLow;
    }

    float outputHigh() const {
        return sampleHigh;
    }

    float outputLow() const {
        return sampleLow;
    }

    float output() const {
        return sampleHigh + sampleLow;
    }

    float deviation() const {

        return
            angleValue /
            (PI / 4.0f) *
            100.0f;
    }

    float triggerLevel() const {
        return
            trigger.high() /
            (PI / 4.0f);
    }

    void setTrigger(float value) {

        float threshold =
            value * (PI / 4.0f);

        trigger.low(-threshold);
        trigger.high(+threshold);
    }
};

#endif
