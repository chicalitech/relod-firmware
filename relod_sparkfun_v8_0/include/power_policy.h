#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace relod {
constexpr uint64_t kReportIntervalMs = 3ULL * 60 * 60 * 1000;
constexpr uint64_t kLidRetryMs = 15000;
constexpr uint8_t kMaxLidRetries = 6;

// A coherent temperature/RH pair for the screen, independent of distance.
// Keep old valid values on sensor failure without advancing their timestamp.
struct ClimateCache {
  float temperature, humidity;
  uint64_t capturedMs;
  bool valid;
  bool update(float temp, float rh, uint64_t now) {
    if (!std::isfinite(temp) || !std::isfinite(rh) || rh < 0 || rh > 100) return false;
    temperature = temp;
    humidity = rh;
    capturedMs = now;
    valid = true;
    return true;
  }
};

struct Vector3 { float x, y, z; };
struct LidConfig {
  Vector3 closedGravity;
  float maxTiltDegrees;
  float maxMovementG;
  uint32_t stableMs;
  uint32_t maxSampleGapMs = 200;
};

inline bool horizontal(Vector3 value, const LidConfig& config) {
  const float magnitude = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  const auto ref = config.closedGravity;
  const float refMagnitude = std::sqrt(ref.x * ref.x + ref.y * ref.y + ref.z * ref.z);
  if (!std::isfinite(magnitude) || !std::isfinite(refMagnitude) ||
      magnitude < 0.90f || magnitude > 1.10f || refMagnitude < 0.1f) return false;
  const float cosine = (value.x * ref.x + value.y * ref.y + value.z * ref.z) /
                       (magnitude * refMagnitude);
  return cosine >= std::cos(config.maxTiltDegrees * 0.01745329252f);
}

// Stability is relative to the start of the window, not just the last sample:
// a slowly tilting lid must not pass because each individual step was small.
class LidGate {
 public:
  explicit LidGate(LidConfig config) : config_(config) {}
  void reset() { tracking_ = false; ready_ = false; }
  bool observe(Vector3 value, uint32_t nowMs, bool readOk = true) {
    if (!readOk || !horizontal(value, config_)) { reset(); return false; }
    if (tracking_ && static_cast<uint32_t>(nowMs - lastObservation_) > config_.maxSampleGapMs) reset();
    lastObservation_ = nowMs;
    const float dx = value.x - anchor_.x, dy = value.y - anchor_.y, dz = value.z - anchor_.z;
    if (!tracking_ || dx * dx + dy * dy + dz * dz > config_.maxMovementG * config_.maxMovementG) {
      anchor_ = value;
      since_ = nowMs;
      tracking_ = true;
      ready_ = false;
      return false;
    }
    ready_ = static_cast<uint32_t>(nowMs - since_) >= config_.stableMs;
    return ready_;
  }
 private:
  LidConfig config_;
  Vector3 anchor_{};
  uint32_t since_ = 0;
  uint32_t lastObservation_ = 0;
  bool tracking_ = false;
  bool ready_ = false;
};

inline uint64_t nextSleepMs(uint64_t now, uint64_t reportDue, bool needsFollowup,
                            uint8_t& retries) {
  uint64_t remaining = reportDue > now ? reportDue - now : kReportIntervalMs;
  if (needsFollowup && retries < kMaxLidRetries) {
    ++retries;
    if (remaining > kLidRetryMs) remaining = kLidRetryMs;
  }
  return remaining < 1000 ? 1000 : remaining;
}

inline bool retryableHttp(int code) {
  return code < 0 || code == 408 || code == 429 || (code >= 500 && code <= 599);
}
inline bool successfulHttp(int code) { return code >= 200 && code < 300; }

// Strict numeric versions, one to three components. Reject suffixes, empty
// components, overflow and trailing garbage instead of accidentally upgrading.
inline bool parseVersion(const char* text, uint32_t (&parts)[3]) {
  if (!text || !*text) return false;
  parts[0] = parts[1] = parts[2] = 0;
  for (size_t i = 0; i < 3; ++i) {
    if (*text < '0' || *text > '9') return false;
    while (*text >= '0' && *text <= '9') {
      const uint32_t digit = *text++ - '0';
      if (parts[i] > (std::numeric_limits<uint32_t>::max() - digit) / 10) return false;
      parts[i] = parts[i] * 10 + digit;
    }
    if (!*text) return true;
    if (*text++ != '.' || i == 2) return false;
  }
  return false;
}
inline bool newerVersion(const char* candidate, const char* current) {
  uint32_t a[3], b[3];
  if (!parseVersion(candidate, a) || !parseVersion(current, b)) return false;
  for (size_t i = 0; i < 3; ++i) {
    if (a[i] != b[i]) return a[i] > b[i];
  }
  return false;
}
inline bool sha256Hex(const char* digest) {
  if (!digest) return false;
  for (size_t i = 0; i < 64; ++i) {
    const char c = digest[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
  }
  return digest[64] == '\0';
}

template <typename T, size_t Capacity> struct Queue {
  // Keep this aggregate trivial: an RTC object must not run a constructor
  // that clears the queue on each deep-sleep boot. Callers value-initialize it.
  T items[Capacity];
  uint8_t count;
  uint32_t dropped;
  void pop() {
    if (!count) return;
    for (size_t i = 1; i < count; ++i) items[i - 1] = items[i];
    --count;
  }
  void push(const T& item) {
    if (count == Capacity) { pop(); ++dropped; }
    items[count++] = item;
  }
};
}  // namespace relod
