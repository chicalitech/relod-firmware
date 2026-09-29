#include "power_policy.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <type_traits>

static_assert(std::is_trivial<relod::Queue<int, 4>>::value, "Queue must survive RTC wake");

// Compile-time checks run even with the ESP32 cross-compiler: no device needed.
constexpr bool openingEventsPreserveBriefTilts() {
  relod::LidEvents events{};
  if (events.observe(true, true)) return false; // Booting tilted is not an opening.
  events.confirmClosed();
  if (events.observe(false, true)) return false; // Bad reads / non-gravity ignored.
  if (events.observe(true, false)) return false; // Flat is not an opening.
  if (!events.observe(true, true)) return false; // First observed tilt is retained.
  if (events.observe(true, false)) return false; // Briefly flat does not re-arm.
  if (events.observe(true, true)) return false; // One event while still open.
  auto afterSleep = events;
  if (afterSleep.observe(true, true)) return false; // RTC state survives follow-up.
  afterSleep.confirmClosed(); // Re-arm only after the stable-lid gate passes.
  return afterSleep.observe(true, true);
}
static_assert(std::is_trivial<relod::LidEvents>::value, "Opening state must survive RTC wake");
static_assert(openingEventsPreserveBriefTilts(), "Brief opening must survive later flat samples and timer follow-ups");

static_assert(relod::chargerState(0, 0) == relod::ChargerState::Unplugged);
static_assert(relod::chargerState(174, 3500) == relod::ChargerState::Charging);
static_assert(relod::chargerState(3500, 1047) == relod::ChargerState::Full);
static_assert(relod::chargerState(174, 3100) == relod::ChargerState::Charging); // ADC high may clip.
static_assert(relod::chargerState(3300, 3300) == relod::ChargerState::Unknown);
static_assert(relod::chargerState(1800, 1047) == relod::ChargerState::Unknown);
static_assert(relod::chargerWakeLevel(174) == 1);
static_assert(relod::chargerWakeLevel(3500) == 0);
static_assert(relod::chargerWakeLevel(1047) == -1); // Never depend on ambiguous digital LOW.
static_assert(relod::chargerRetryMs(0) == 10000);
static_assert(relod::chargerRetryMs(4) == 160000);
static_assert(relod::chargerRetryMs(5) == 300000);
static_assert(relod::chargerRetryMs(255) == 300000);
static_assert(relod::sleepUntilMs(50, 100) == 50);
static_assert(relod::sleepUntilMs(100, 100) == 1);
static_assert(relod::sleepUntilMs(101, 100) == 1); // No unsigned underflow after slow refresh.
static_assert(relod::chargerRefreshOnly(false, true, false, 50, 100, 75));
static_assert(!relod::chargerRefreshOnly(false, true, true, 50, 100, 75)); // Motion still handled.
static_assert(!relod::chargerRefreshOnly(false, true, false, 75, 100, 75)); // Lid retry due.
static_assert(!relod::chargerRefreshOnly(false, true, false, 100, 100, 200)); // Report due.
static_assert(!relod::chargerRefreshOnly(true, true, false, 50, 100, 75)); // Cold-boot setup preserved.

int main() {
  static_assert(std::is_trivial<relod::ClimateCache>::value, "Climate cache must survive RTC wake");
  relod::ClimateCache climate{};
  assert(!climate.valid);
  assert(!climate.update(NAN, 50, 100));
  assert(!climate.valid); // Missing first reading must not appear as zero.
  assert(climate.update(24.5f, 48, 200));
  assert(climate.valid && climate.capturedMs == 200);
  assert(!climate.update(25, NAN, 300));
  assert(!climate.update(INFINITY, 50, 300));
  assert(!climate.update(25, -1, 300));
  assert(!climate.update(25, 101, 300));
  assert(climate.temperature == 24.5f && climate.humidity == 48 && climate.capturedMs == 200);
  assert(climate.update(-5, 0, 400));
  assert(climate.update(30, 100, 500));
  assert(climate.temperature == 30 && climate.humidity == 100 && climate.capturedMs == 500);

  const relod::LidConfig config{{0, 0, 1}, 12, 0.035f, 750};
  assert(relod::horizontal({0, 0, 1}, config));
  assert(!relod::horizontal({0, 0, -1}, config)); // Upside-down lid is not closed.
  assert(!relod::horizontal({1, 0, 0}, config));
  assert(!relod::horizontal({0, 0, 0}, config)); // Free fall / missing data.
  assert(!relod::horizontal({NAN, 0, 1}, config));
  assert(!relod::horizontal({0, 0, INFINITY}, config));
  assert(!relod::horizontal({0, 0, 1.2f}, config)); // Acceleration is not gravity.
  assert(relod::horizontal({0.17f, 0, 0.985f}, config)); // About 10 degrees.
  assert(!relod::horizontal({0.26f, 0, 0.966f}, config)); // About 15 degrees.
  assert(relod::horizontal({1, 0, 0}, {{1, 0, 0}, 12, 0.035f, 750}));
  assert(!relod::horizontal({0, 0, 1}, {{0, 0, 0}, 12, 0.035f, 750}));

  relod::LidGate gate(config);
  for (uint32_t ms = 0; ms < 750; ms += 50) assert(!gate.observe({0, 0, 1}, ms));
  assert(gate.observe({0, 0, 1}, 750));
  assert(!gate.observe({0.1f, 0, 0.995f}, 800)); // Movement resets stability.
  for (uint32_t ms = 850; ms < 1550; ms += 50) assert(!gate.observe({0.1f, 0, 0.995f}, ms));
  assert(gate.observe({0.1f, 0, 0.995f}, 1550));
  assert(!gate.observe({0.1f, 0, 0.995f}, 1600, false)); // Read errors fail closed.
  for (uint32_t ms = 1700; ms < 2450; ms += 50) assert(!gate.observe({0.1f, 0, 0.995f}, ms));
  assert(gate.observe({0.1f, 0, 0.995f}, 2450));
  // A long gap in observations cannot prove the lid stayed still.
  assert(!gate.observe({0.1f, 0, 0.995f}, 3000));
  gate.reset();
  for (uint32_t elapsed = 0; elapsed < 750; elapsed += 50)
    assert(!gate.observe({0, 0, 1}, uint32_t(0xFFFFFF00U + elapsed)));
  assert(gate.observe({0, 0, 1}, uint32_t(0xFFFFFF00U + 750))); // millis wrap.
  gate.reset();
  for (uint32_t ms = 0; ms <= 1000; ms += 100)
    assert(!gate.observe({ms * 0.0001f, 0, 1}, ms)); // Slow cumulative movement.

  uint8_t retries = 0;
  for (int i = 0; i < 6; ++i) assert(relod::nextSleepMs(0, 10800000, true, retries) == 15000);
  assert(retries == 6);
  assert(relod::nextSleepMs(0, 10800000, true, retries) == 10800000);
  retries = 0;
  assert(relod::nextSleepMs(9000, 10000, true, retries) == 1000);
  assert(relod::nextSleepMs(0, 10000, false, retries) == 10000);

  assert(relod::successfulHttp(200) && relod::successfulHttp(204));
  assert(!relod::successfulHttp(301) && !relod::successfulHttp(500));
  assert(relod::retryableHttp(-1) && relod::retryableHttp(408));
  assert(relod::retryableHttp(429) && relod::retryableHttp(503));
  assert(!relod::retryableHttp(400) && !relod::retryableHttp(401));

  assert(relod::newerVersion("8.0.1", "8.0.0"));
  assert(relod::newerVersion("9", "8.0.0"));
  assert(!relod::newerVersion("8.0", "8.0.0"));
  assert(!relod::newerVersion("7.9.9", "8.0.0"));
  for (const char* malformed : {"9evil", "9.", "9..0", "9.0.0.1", "9.0.0-beta", "", "999999999999"}) {
    assert(!relod::newerVersion(malformed, "8.0.0"));
  }
  assert(relod::sha256Hex("0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"));
  assert(!relod::sha256Hex("0123"));
  assert(!relod::sha256Hex("g123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"));
  assert(!relod::sha256Hex("0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0"));
  relod::Queue<int, 4> queue{};
  queue.pop();
  for (int i = 1; i <= 5; ++i) queue.push(i);
  assert(queue.count == 4 && queue.dropped == 1 && queue.items[0] == 2);
  queue.pop();
  assert(queue.count == 3 && queue.items[0] == 3);
  std::puts("PASS: lid qualification, stability, sleep policy, HTTP outcomes, OTA metadata, queue retention");
}
