# Relod v8.0.0 — battery and lid qualification candidate

This is a separate SparkFun Thing Plus ESP32-C6 project. v7 and the repository's
production XIAO sketch are unchanged. Open **this folder** with the VS Code
**ESP32 Arduino** profile, then choose the `sparkfun_c6` environment.

## Behavior

1. Read the BMA400 first, in normal mode at 100 Hz. Require acceleration magnitude
   between 0.90 and 1.10 g, within 12 degrees of the configured closed-lid gravity
   vector, and movement no greater than 0.035 g relative to the start of a
   continuous 750 ms stability window, with no sample gap over 200 ms. Give this check at most 2 seconds.
2. Only after qualification, initialize the VL53L5CX. Requalify after its firmware
   initialization, check orientation/movement while waiting for a frame, and
   check again after reading it. Reject inadequate frames rather than fabricating
   an all-zero measurement. At least 8 of 64 zones must have status 5 or 9, a
   detected target, and distance in the open interval (0, 3500) mm.
3. Stop and sleep the imager immediately, including partial failure paths.
   Read climate once per ordinary wake, independently of lid/range success; read
   battery once per ordinary wake as well. Do not quick-start the
   fuel gauge on routine ESP32 wakes.
4. Retain valid samples in a four-entry RTC-memory queue. Send at most two POST
   attempts per wake, count only 2xx as success, and retain unsuccessful samples.
5. Check OTA metadata at most once per 24 hours after successful reporting and
   an empty queue. Require a reported SOC of at least 30%, voltage at least
   3.65 V, a strictly newer numeric version, HTTPS download URL, matching board,
   and a valid matching SHA-256. Missing battery data or metadata skips OTA.
6. Shut down Wi-Fi before the final display refresh. Refresh once on ordinary
   wakes, skip unchanged display content, then call `display(true)` to send the
   e-ink controller into sleep. A bounded BUSY handler suppresses display work
   for 24 hours after a timeout. Reset/power cycling clears that suppression.
7. Put the BMA400 into low-power mode and arm motion wake only if its interrupt
   line is inactive. A timer always remains as a fallback.

The three-hour scheduled-report interval remains. A failed lid/range check gets
up to six follow-ups, 15 seconds apart, with motion wake disabled (charger wake remains available). This avoids both a long awake
wait and repeated GPIO wake storms. If the lid stays open beyond that window,
the next opportunity is a later motion wake or the next scheduled report. A
persistently asserted/faulty interrupt leaves timer wake as the recovery path;
there is no promise of immediate closure detection in that fault condition.
Qualification attempts impose a 30-second GPIO-wake cooldown; successful samples extend it from completion. Motion reports
do not push the regular three-hour deadline farther into the future.

Tilted/moving/missing-sensor ordinary wakes do not initialize Wi-Fi or start
ranging. Last valid local data remains available and the screen shows its age.
On a cold boot with an unqualified lid, the imager is initialized once only to
command sleep: a newly powered sensor may otherwise remain in high-power idle.
No ranging is started in this exception. No measurement POST is fabricated for a skipped range. Timed follow-ups also
retry a range-data failure; the screen distinguishes it from an orientation wait.

## Calibrate before using inventory readings

`LID_CLOSED_X/Y/Z` in `platformio.ini` is calibrated to **(-0.008, -0.016, -1.037)**
for the assembled RELOD-6514 lid, measured flat and closed on 2026-09-21. Its
accelerometer faces -Z upward; the previous +Z reference rejected this position.
This is specific to this mounting, not automatic calibration. Recalibrate other
installations. The BMA400 must be attached rigidly to the lid.

1. Place the assembled lid in its normal closed, horizontal position.
2. Build `sparkfun_c6_debug` and inspect the printed `Lid acceleration` vector.
3. Use the stationary measured vector for `LID_CLOSED_X/Y/Z`. The gate normalizes
   the reference, so a reading like `(0.01, -0.02, -0.99)` is usable directly.
4. Verify closed passes; tilted, upside-down, moving, and disconnected sensors
   fail. Repeat while lifting the lid during imager initialization and ranging.
5. Tune angle/stability in `kLidConfig` only after observing real lid handling.

A horizontal removed lid can pass. This is an orientation/stability gate, not
proof of mechanical closure. Use a reed/Hall switch if closure must be certain.
Continuous acceleration that resembles gravity cannot be distinguished by an
accelerometer alone. Time-of-flight validity thresholds also need validation
with the actual container and contents.

## Wi-Fi setup and recovery

Saved credentials get an 8-second connection timeout. There is no second 10-second
wait. Deep-sleep wakes never open the configuration hotspot. A deliberate RESET
or power cycle permits the original `relod-xxxx` / `password` setup portal if the
connection fails, even when the lid is tilted or sensors are absent. Its timeout
is 180 seconds; connect at `192.168.4.1` if captive-portal discovery fails. After
timeout the device sleeps; reset again to deliberately reopen provisioning.

Only ordinary reporting requires a valid range. Cold boot may still use Wi-Fi
for setup/recovery without taking a range. Wi-Fi uses modem sleep after connecting
and does not force maximum TX power. Validate connection reliability and energy
per report on the real router before adopting more aggressive radio tuning.

HTTP connection, TLS handshake, and read timeouts are 4 s, 5 s, and 3 s. The normal
reporting loop does not start another POST after 18 s of its 30 s planning budget.
These are stage limits and a scheduling budget, not a hard preemptive wall-clock
deadline for every DNS/header parsing failure inside the network libraries.
There is no delay after the final failed attempt, no error-triggered reboot loop,
and no repeated live retry for HTTP 429. Other permanent 4xx responses also end
the wake's attempts. Failed uploads remain queued; invalid old entries eventually
age out through the documented overflow policy rather than an infinite flush.

Time is kept through deep sleep using RTC-backed system time. NTP is attempted
daily after a successful sync, with a maximum 1.2-second foreground wait. Failed
time sync is eligible again after six hours. The independent RTC counter drives
scheduling and sample ages, so NTP clock corrections do not change those timers.
The opened timestamp records an observed tilted GPIO wake, not every reset or
timer wake. It is an approximate observed opening event; it is not a lid switch.

## Payload and offline retention

Existing fields are preserved for accepted measurements, including 64 distance
zones and the existing zone ordering. Unusable individual zones are zero as a
legacy convention, but an all-zero/low-quality frame is never accepted.

| Additional field | Meaning |
| --- | --- |
| `schema_version` | `2` for this candidate |
| `measurement_id` | MAC + random RTC-session ID + sequence; stable across retry |
| `measurement_sequence` | Increments only for accepted samples in this RTC session |
| `measurement_age_s` | Age of this specific sample at payload generation |
| `measured_at_unix_s` | Capture time inferred from RTC age and current synchronized time; null if time unknown |
| `distance_fresh` | True only for the most recent accepted sample less than 60 seconds old; false for older/backlogged data |
| `distance_valid_count` | Number of usable zones in this frame |
| `lid_horizontal`, `lid_stable` | True for the accepted frame, not a claim about lid state when a queued frame is uploaded |
| `skipped_measurements` | Number of failed qualification/range attempts this RTC session |
| `queue_dropped` | Number of oldest unsent frames discarded because the four-entry queue filled |
| `queue_depth` | Entries waiting before this POST |
| `wifi_connect_ms`, `wifi_rssi_dbm` | Current wake's Wi-Fi connection duration and RSSI |
| `previous_http_status`, `previous_post_ms` | Previous completed POST attempt, possibly earlier in the same wake |
| `previous_awake_ms` | Previous completed wake duration |

The device sends `Idempotency-Key` as well as `measurement_id`. **The server must
implement deduplication** to make retries exactly-once at the application level;
the header alone does not do that. Servers should use capture time/age for delayed
samples. An older server may accept the added fields but ignore their semantics,
so verify ingestion/UI behavior before deploying backlog reporting broadly.

The queue and last valid reading survive deep sleep only. RESET, power loss,
brownout, and OTA reboot clear this session. At capacity, the oldest unsent sample
is dropped and the loss counter increments. This is bounded memory, not durable
storage or a guarantee against data loss.

## OTA safeguards and remaining transport limitation

Metadata must supply `version`, `url`, `sha256` (or `hash`), and
`board: "sparkfun_esp32c6_thing_plus"`. Numeric versions have one to three
components; suffixes/malformed versions are rejected. Force/downgrade metadata
does not override the newer-version rule. Images must fit the inactive OTA slot.
The download aborts on hash mismatch, stream errors, a 5-second progress stall,
or 120 seconds of streaming. Boot selection is committed only after verification.

Like v7, this candidate still uses `setInsecure()` for TLS. SHA-256 checks transfer
integrity against metadata; it does not authenticate an untrusted metadata
server. Validate a CA/bundle or signed manifest before production OTA rollout.
No server metadata, remote deployment, board flash, or hardware jumper is changed
by building this project.

## Wiring and build

Keep v7 production wiring: EPD MOSI=4, CS=5, DC=17, RESET=16, BUSY=18, SCK=19;
SRAM CS is unused and must remain deselected. BMA400 INT1 goes to GPIO3. Do not
insert a microSD card without resolving the board's shared IO18/IO19 connections.
GPIO9 BUSY wiring is only available in the explicit `sparkfun_c6_soldered_test`
environment and remains a boot-strap hazard. The normal build uses GPIO18.

```powershell
C:\pio\penv\Scripts\pio.exe run -e sparkfun_c6
C:\pio\penv\Scripts\pio.exe run -e sparkfun_c6_debug
```

Use a verified ESP32 USB port for upload; COM4/COM5 on this PC were Bluetooth
ports. The 16 MB partition table retains two OTA slots. Use USB for initial v8
hardware validation; do not publish it through the fleet metadata endpoint.

Portable policy tests compile with any C++17 host compiler:

```text
c++ -std=c++17 -Wall -Wextra -Werror -I include tests/power_policy_test.cpp -o policy-test
./policy-test
```

`../scripts/check` can be run with this project as the current directory and
the compatible `C:\pio\penv\Scripts` Core on PATH. It builds the current project.

## Staged hardware validation / rollout plan

1. Build and run host policy tests. Check a single unit over USB with OTA metadata
   disabled or no matching v8 board metadata; verify serial messages and readings.
2. Calibrate closed orientation. Test tilt, upside-down, slow movement, missing
   BMA400, missing VL53L5CX, movement during ranging, and a lid left open longer
   than the 90-second follow-up window. Check the 30-second movement cooldown.
3. Turn the router off and simulate server timeouts/4xx/5xx/429. Verify bounded
   attempts, no routine setup AP, queue overflow counters, and correct capture
   timestamps/IDs on reconnect. Verify backend dedup and stale-data handling.
4. Measure current from the battery across a complete wake and sleep, including
   Wi-Fi failures, e-ink BUSY failure, and motion interrupt held high. Verify the
   imager stops, EPD sleeps, and BMA400 remains able to wake the device.
5. Test valid OTA on one recoverable unit, then hash mismatch, wrong board, old
   version, oversized/truncated image, low battery, and interrupted download.
6. Compare charge per report and sleep current against v7 at the same cadence.
   The PWR LED and breakout leakage may dominate; inspect SparkFun's hardware
   guidance separately before any jumper or power-gating modifications.
7. Resolve TLS authentication and server timestamp/dedup handling, obtain owner
   acceptance of provisioning/retry behavior, then stage a small rollout. Keep
   v7 available for manual recovery. No battery-life duration is claimed without
   battery capacity and measured whole-device current.

## Software validation completed

- Host C++17 policy tests passed with warnings treated as errors: orientation,
  upside-down/invalid acceleration, stability, sensor observation gaps, clock
  wrap, retry limits, queue overflow/RTC triviality, and OTA version/hash format.
- `sparkfun_c6` release build passed: 49,896 bytes RAM; 1,431,462 bytes app sections
  out of the 6,553,600-byte OTA slot (21.8%).
- `sparkfun_c6_debug` build passed: 49,896 bytes RAM; 1,487,480 bytes app sections
  out of the same slot (22.7%).
- The repository check script passed when run from the v8 folder. Its Git Bash
  invocation emitted an ESP-IDF installer MSys warning; the final native
  PowerShell builds also passed without that environment warning.
- Source equivalence with the host-tested policy was verified. v7 and the
  production sketch have no changes in this task.
- Hardware calibration, current measurements, actual POST/OTA failure injection,
  and flashing have not been performed.

## Display readability update

The screen labels battery percentage explicitly and uses larger proportional
fonts for climate, lid status, Wi-Fi, and contents. Setup reads `Wi-Fi setup`
and `Join relod-xxxx`; the hotspot name appears once. Long network names are
ellipsized to the panel width. The smaller detail rows retain the opened time
and separate ages for the distance and temperature/RH readings.

Temperature/RH are sampled once per normal wake before networking, even if the
lid is tilted or distance fails. The early motion-cooldown return still skips
sensor/display work. The most recent valid climate pair survives deep sleep;
a failed sensor read retains that pair with its original age, or shows `--`
if no valid pair exists. POSTs use only the current wake's climate readings,
never the retained fallback. No extra Wi-Fi sessions, continuous sensor polling,
or periodic screen-refresh wakeups are added.

Display update validation (2026-09-16): host regression tests cover initial
missing data, valid climate updates, NaN/infinity, humidity bounds, retention
of the previous pair and timestamp, and RTC triviality. Release/check-script
and debug builds passed. Pixel previews using the installed GFX font bitmaps
checked setup, connected, stale-reading, missing-sensor, 100% battery, and long
SSID cases against 250 x 122 bounds. Physical display appearance and the added
climate-read energy on unqualified wakes still need checking on the assembled lid.

The header uses white text/icons on black: device ID on the left, Wi-Fi and a
filled battery outline on the right, with no numeric percentage. A slash marks offline
Wi-Fi; setup mode combines the status and hotspot name on one line, for
example `Wi-Fi setup: relod-6514`. This is the status
at the last refresh, not a continuously connected radio or a live signal meter.
`Wi-Fi sleeping` means the radio was deliberately left off on this wake;
`Wi-Fi offline` means a connection was attempted but failed. Neither message
means saved credentials were erased.
Unknown battery data shows `?` inside the outline.

### Charger-triggered display updates

The lightning bolt now follows the external MCP73833 charger status rather
than the MAX17048 charge-rate estimate. J3 yellow/STAT1 is connected to GPIO2,
green/STAT2 to GPIO1, and red/PG to GPIO0. These measured voltages were taken
relative to ground with the GPIO wires connected:

| Cable / charger state | Yellow | Green | Header |
| --- | --- | --- | --- |
| Unplugged | approximately 0 V | approximately 0 V | Battery level, no bolt |
| Charging | 0.174 V | 3.5 V | Battery level and bolt |
| Charge complete | 3.5 V | 1.047 V | Full battery, no bolt |

Detection averages calibrated ADC samples and requires three matching states
50 ms apart. Values outside these combinations are treated as unknown, with
no charging bolt. Classification thresholds are specific to this assembly:
low <=350 mV, high >=2600 mV, and green-on <=1500 mV when yellow is high.

ESP32-C6 EXT1 wake levels are selected separately for each status pin. A pin
below 600 mV watches for HIGH; a pin at or above 2600 mV watches for LOW.
Intermediate voltages are excluded. Thus, at full charge only yellow is armed;
the measured 1.047 V green signal is not treated as a reliable digital level.
GPIO3 retains motion wake detection. No internal pull-up is enabled on the
charger pins. Red/PG is not needed to distinguish these measured states.

A charger-only wake reads the gauge, refreshes changed display content, and
returns to sleep without starting Wi-Fi, POST, OTA, distance, or climate reads.
The last measurement and network snapshot remain visible. Existing report and
lid-retry deadlines are preserved; a simultaneous motion event or due report
uses the normal path. Charge completion also triggers an update. E-ink refresh
latency still applies, and cable changes during blocking provisioning/network
work are reflected when that work reaches its next display update.

Transitions during a refresh get a short follow-up. Ambiguous inputs, failed
wake arming, or repeated rapid wakes use bounded retries, backing off from
10 seconds to at most five minutes; unchanged content does not refresh the
panel. Charger wakes remain available during lid-retry sleeps, even when
motion wakes are temporarily disabled. Display-fault backoff still applies.
The existing LED-voltage POST fields and schema are unchanged.

The board currently has direct status-to-GPIO wiring. These thresholds are a
functional interpretation of the supplied measurements, not electrical
protection or proof of voltage limits in every power state. See the
[MCP73833 datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP73833-4-Stand-Alone-Linear-Li-Ion-Li-Polymer-Charge-Management-Controller-DS20002005.pdf)
and [ESP32-C6 datasheet](https://documentation.espressif.com/esp32-c6_datasheet_en.html).

Before wider rollout, flash this assembly and verify charging, charge complete,
unplugging from both states, and a simultaneous lid lift. Confirm the wake log
shows the expected state and that ordinary charger events do not send POSTs.
Check sleep current with and without the cable; hardware validation is pending.
Compile-time assertions cover the supplied voltage combinations, ambiguous green
wake exclusion, retry limits, expired deadlines, and motion/report priority.
The `sparkfun_c6` release build passed via `scripts/check`.


### Measurement and opening timestamps

The screen labels these separate events explicitly:
- `Last measured: YYYY-MM-DD HH:MM` is the latest valid distance sample in the
  configured local timezone, not the server upload time. An unsuccessful
  lid/range check keeps the previous sample time. Before a valid sample it says
  `Not yet`; before clock synchronization it shows the sample age with `no clock`.
- `Lid opened: YYYY-MM-DD HH:MM` is the last recorded opening event. Its saved
  value can survive flashing and need not match the measurement date.

These rows replace the former Range/T-RH age and Opened rows. Temperature and
humidity still show the latest valid cached climate pair, collected separately
from distance. The timestamps are snapshots at refresh, not live clocks.


### Lid-opening event tracking

A confirmed stable, closed orientation arms opening detection. The first valid
observed tilt records one opening immediately, before range qualification ends.
Returning to a stable closed position arms the next event. Timer follow-ups and
range failures do not suppress this tracking, and a motion wake during the
30-second reporting cooldown takes one acceleration reading to retain an opening.
Opening-event tracking adds no wakeups, Wi-Fi connections, or changes to the report schedule.
With a valid clock, opening time is persisted even when Wi-Fi is skipped.

An opening entirely between sensor observations can still be missed, and motion
wake must function through BMA400 INT1 to GPIO3. A horizontal lift is not detectable
as a tilt; this remains an orientation estimate, not a mechanical closure sensor.
The serial `Lid event` and `Lid cooldown` lines expose wake cause and saved opening
time for hardware verification. `sparkfun_c6_debug` adds startup time to attach USB.
After flashing, let the lid qualify closed, then lift/tilt it and return it flat.
Verify a new timestamp, no repeated events while held tilted, and another event
on a second opening. Hardware confirmation of this change is still required.
