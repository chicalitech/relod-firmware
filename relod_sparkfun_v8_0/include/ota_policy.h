#pragma once
#include <cmath>
#include <cstdint>

namespace relod {
// Call only after charger-only and motion-cooldown wakes have returned.
inline bool otaOpportunity(uint64_t now, uint64_t due, bool coldBoot,
                           bool reportDue, bool measured, bool batteryReady,
                           float soc, float voltage) {
  return now >= due && (coldBoot || reportDue || measured) && batteryReady &&
         std::isfinite(soc) && std::isfinite(voltage) && soc >= 30 && voltage >= 3.65f;
}
} // namespace relod
