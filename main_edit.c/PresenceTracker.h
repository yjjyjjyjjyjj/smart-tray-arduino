#ifndef PRESENCE_TRACKER_H
#define PRESENCE_TRACKER_H
#include <stdint.h>

// Fixed-size, allocation-free logic shared by the AVR firmware and host tests.
struct StableSamples {
  int32_t low, high, mean;
  uint32_t since, last;
  uint8_t count;
  void clear() { count = 0; }
  bool add(int32_t value, uint32_t now, int32_t tolerance,
           uint8_t requiredSamples = 20, uint32_t requiredMs = 1800) {
    if (!count || uint32_t(now - last) > 650 ||
        value < low - tolerance || value > high + tolerance ||
        (value < low ? high - value : value - low) > tolerance) {
      low = high = mean = value; since = now; count = 1;
    } else {
      if (value < low) low = value;
      if (value > high) high = value;
      if (count < 30) ++count;
      mean += (value - mean) / count;
    }
    last = now;
    return count >= requiredSamples && uint32_t(now - since) >= requiredMs;
  }
};

struct PresenceTracker {
  int32_t zero;                 // RAM-only working zero; reference remains in EEPROM config
  int32_t peakRaw;
  int32_t samples[3];
  int16_t weight, peak;
  uint32_t last, detectSince;
  uint8_t sampleCount, cursor, detectCount;
  bool present, ready;
  StableSamples stable;
  StableSamples releaseStable;

  static int32_t magnitude(int32_t x) { return x < 0 ? -x : x; }
  static int32_t median(int32_t a, int32_t b, int32_t c) {
    if (a > b) { int32_t t=a; a=b; b=t; }
    if (b > c) { int32_t t=b; b=c; c=t; }
    return a > b ? a : b;
  }
  void interrupt() { sampleCount = cursor = detectCount = 0; stable.clear(); releaseStable.clear(); }
  void reset(int32_t reference) {
    zero = reference; peakRaw = 0; weight = peak = 0; present = ready = false; last = 0;
    interrupt();
  }
  // raw span corresponding to 1 g, bounded below for coarse calibration factors.
  static int32_t stabilityTolerance(int32_t scale) {
    int32_t t = magnitude(scale) / 10;
    return t > 2 ? t : 2;
  }
  // Bootstrap calibration has no trustworthy counts/gram conversion yet.
  static int32_t calibrationTolerance(int32_t scale, bool calibrated) {
    return calibrated ? stabilityTolerance(scale) : 512;
  }
  static int16_t validRelease(int16_t detect, int16_t release) {
    return release > 0 && release < detect ? release : detect/2;
  }
  bool update(int32_t raw, uint32_t now, int32_t reference, int32_t scale,
              int16_t detect, int16_t release, int16_t deadzone) {
    if (!scale || detect < 10) return false;
    if (sampleCount && uint32_t(now-last) > 650) interrupt();
    last = now;
    samples[cursor] = raw; cursor = (cursor+1)%3;
    if (sampleCount < 3) ++sampleCount;
    int16_t band = deadzone;
    if (band > release) band = release;
    if (band > detect/3) band = detect/3;
    if (band < 1) band = 1;
    // Limit noise to one quarter of the empty band, rather than an unrelated
    // fixed 1 g. Default 12 g band permits a 3 g peak-to-peak stable window.
    int32_t tolerance = (magnitude(scale) * (band/4)) / 100;
    if (tolerance < 2) tolerance = 2;
    if (sampleCount < 3) { stable.add(raw, now, tolerance); return false; }
    ready = true;
    int32_t filtered = median(samples[0], samples[1], samples[2]);
    // HX711 signed 24-bit range: difference * 100 fits int32_t.
    int32_t dg = (filtered-zero)*100/scale;
    // A large negative return must not latch presence forever. Permit up to
    // 1% of the observed load excursion as noise ONLY below the empty band.
    // Positive partial loads still use the original narrow stability rule.
    if (present && dg < -band && peakRaw/100 > tolerance) tolerance = peakRaw/100;
    bool settled = stable.add(raw, now, tolerance);
    weight = dg < -30000 ? -30000 : dg > 30000 ? 30000 : (int16_t)dg;
    // A small negative residual is not positive load. Keep the positive band
    // narrow; both directions remain bounded by the original empty reference.
    bool nearEmpty = dg <= band && dg >= -int32_t(band)*2 &&
        magnitude((stable.mean-reference)*100/scale) <= int32_t(band)*2;
    bool changed = false;
    if (!present) {
      if (dg >= detect) {
        if (!detectCount) detectSince = now;
        if (detectCount < 255) ++detectCount;
        if (detectCount >= 4 && uint32_t(now-detectSince) >= 450) {
          present = true; peak = weight; peakRaw = magnitude(filtered-zero);
          stable.clear(); releaseStable.clear(); changed = true;
        }
      } else detectCount = 0;
    } else {
      if (weight > peak) peak = weight;
      if (dg > 0 && magnitude(filtered-zero) > peakRaw) peakRaw = magnitude(filtered-zero);
      // Keep presence and zero through partial removals, brief dips and sensor faults.
      // Release is independent of whether the residual is safe to store as zero.
      bool releaseCandidate = dg <= band && peak-dg >= detect-band;
      bool released = false;
      if (releaseCandidate) released = releaseStable.add(raw, now, tolerance, 3, 150);
      else releaseStable.clear();
      if (released) {
        present = false; detectCount = 0; peak = 0; peakRaw = 0; changed = true;
        releaseStable.clear();
      }
    }
    // Only stable near-empty samples can move zero. Limit TOTAL travel from the
    // manually confirmed empty reference, not just each individual adjustment.
    if (!present && !detectCount && settled && nearEmpty) {
      zero = stable.mean; weight = 0; stable.clear();
    }
    return changed;
  }
};
#endif
