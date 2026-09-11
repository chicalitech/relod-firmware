#include "power_policy.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <type_traits>

static_assert(std::is_trivial<relod::Queue<int, 4>>::value, "Queue must survive RTC wake");

int main() {
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
