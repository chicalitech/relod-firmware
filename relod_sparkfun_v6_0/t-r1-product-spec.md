# t-r1 — First-Version Product Spec

Target launch: October 1, 2026

## Summary

t-r1 is an intelligent coffee-bean container for people who make coffee at home and want to avoid unexpectedly running out. It estimates remaining supply, sends timely text reminders with a saved purchase link, and warns about poor storage conditions. The hardware MVP exists; the October 1 launch adds the software needed to make it useful at home.

## Problem

People who make coffee at home need enough beans on hand to make coffee when they want it without an unexpected trip to buy more.

Running out can disrupt an entire morning, forcing someone to visit a coffee shop or buy beans before making coffee. Keeping stocked requires remembering to check supplies and allowing enough time to replace them.

## Who this is for

The primary user makes coffee at home, buys beans for personal or household use, and has home Wi-Fi and a phone that receives texts. The first version supports one container, one linked phone number, and one type of coffee at a time.

## What success looks like

Evaluate these outcomes during the first two weeks of use. Numerical targets below are proposed assumptions.

- At least 90% of eligible reminder events send the first text when estimated supply reaches delivery time plus one day.
- No refill cycle produces more than two reorder reminders, and none are sent after the user replies “ordered” until a refill is detected.
- At least 90% of refills are detected without correction, with no false refills during deliberate movement tests.
- Test households report fewer unexpected run-outs than before using t-r1; collect weekly feedback because an empty container alone cannot establish whether a run-out was unexpected.

An eligible reminder event requires a usable estimate, a saved purchase link, a delivery-time setting, and connectivity.

## The experience, step by step

1. **Connect the container.** On first power-on, the lid display guides the owner to join the container’s temporary Wi-Fi network and open its setup page. The owner enters home Wi-Fi credentials and links their phone number. t-r1 then joins home Wi-Fi and begins sending readings to the server.
2. **Describe the coffee.** Through text messages, the owner provides the coffee name, purchase link, bag size, amount added—such as a full or half bag—usual cups consumed per day, and typical delivery time in days. t-r1 also asks whether they want storage-condition texts.
3. **See remaining supply.** The lid shows estimated cups remaining and a depletion date. Initial estimates use the owner’s answers; later estimates reflect observed consumption. Estimates are visibly identified as estimates.
4. **Receive a reorder reminder.** When estimated supply reaches delivery time plus one day, t-r1 texts the coffee name, days remaining, and saved purchase link. For three-day delivery, the first reminder arrives at about four days remaining.
5. **Buy from the store.** The link opens the saved product page. The user completes the purchase with the retailer. t-r1 does not place the order or receive purchase confirmation from the store.
6. **Acknowledge or receive one follow-up.** Replying “ordered” stops reorder reminders until the next detected refill. Otherwise, t-r1 sends one follow-up when estimated supply reaches delivery time—three days remaining in the example.
7. **Refill normally.** t-r1 automatically recognizes a sustained increase in contents, updates its estimate, and starts a new refill cycle without requiring a “refilled” text.
8. **Check storage and battery status.** The lid shows temperature, humidity, and any storage warning. Users who opted in receive a text when an unacceptable condition persists long enough to rule out brief changes. A low-battery warning appears before monitoring stops.

**Edge cases**

- **Movement, tilt, or unusual readings:** Ignore unreliable samples and retain the last trustworthy estimate.
- **Insufficient or stale data:** Show that the estimate is learning or unavailable; withhold uncertain reorder texts.
- **Ignored reminders:** Stop after the two scheduled reminders for that refill cycle.
- **Refill before ordering:** Recalculate supply and discard reminders based on the previous contents.
- **Invalid or unavailable purchase link:** Let the owner replace it by text; do not choose another product automatically.
- **Lost Wi-Fi:** Show the last update time and connection status; after reconnection, reassess current supply before sending a reminder.
- **Rapid depletion that skips a reminder point:** Send at most one currently relevant reminder, never two catch-up texts together.

## Requirements

### Must have for the first version

- Owners can connect home Wi-Fi and link their phone through the guided setup flow.
- Owners can provide and correct coffee details, purchase link, quantity, daily consumption, and delivery time through text.
- The lid shows estimated cups remaining, depletion date, temperature, humidity, and applicable warnings.
- Estimates adjust to observed consumption while excluding unreliable readings.
- Reorder texts follow the delivery-time-plus-one-day rule and include the saved purchase link.
- Owners receive at most one follow-up at the delivery-time threshold.
- Replying “ordered” suppresses reorder reminders until a detected refill.
- Refills update supply automatically without a required manual confirmation.
- Owners can choose whether to receive sustained storage-condition warnings by text.
- Owners can recognize when displayed estimates are stale or unavailable.
- Owners see a low-battery warning before monitoring stops.

### Should have if cheap

- Owners can request their current estimate by text.

### Won’t do now

- t-r1 will not purchase coffee, manage subscriptions, or collect retailer credentials or payment information.
- Reorder messages will not display live prices or add affiliate tracking.
- t-r1 will not infer shipping time from a store page.
- The first version will not provide a separate mobile app or manage multiple containers.

## Non-goals

t-r1 does not guarantee delivery dates, measure coffee freshness or taste, control storage temperature or humidity, or manage supplies for offices and cafés. It does not account for spare bags stored outside the container.

## Assumptions

These choices are proposed and can be challenged:

- October 1 means October 1, 2026; the evaluation period is the following two weeks.
- One phone number controls each container.
- The lid displays both cups remaining and depletion date, subject to legibility.
- Texts use a clickable product link; a styled retailer button is not required.
- Storage-condition texts are off until the owner opts in.
- A storage warning requires 30 continuous minutes outside the agreed range, with one text per continuous episode.
- A refill resets reminder suppression and eligibility; uncertain readings never trigger a text.
- If both reminder thresholds have already been crossed, only one reminder is sent for that cycle.
- The low-battery warning appears on the lid; a battery-warning text is not required for launch.
- The success targets are proposed validation targets, not demonstrated hardware performance.

## Open questions

- **How accurately can sensor readings estimate quantity and cups across bean shapes and fill levels?** The engineer and hardware owner should establish calibration and acceptable error.
- **When is an estimate trustworthy enough to trigger a text?** The engineer should propose a rule using measured sensor performance, with the product owner approving the reminder tradeoff.
- **Which temperature and humidity ranges justify warnings, including “too hot” and “too dry”?** The product owner should confirm these with a qualified coffee-storage source before launch.
- **How long can readings remain unchanged before an estimate becomes stale?** The engineer and hardware owner should resolve this alongside battery-life testing.
- **What battery level triggers the warning with enough time to act?** The hardware owner and engineer should establish this through battery testing.
- **What texting regions, service costs, consent, and opt-out behavior are required?** The engineer and product owner should resolve this with the selected messaging service.
- **How many households will launch include, and what battery life is acceptable?** The product owner and hardware owner should decide before launch.

## Notes for the implementer

**Inputs:** Distance-based content readings, temperature, humidity, movement readings, battery status, timestamps, and the owner’s setup answers and text replies.

**Outputs:** Lid status, supply estimates, storage and low-battery warnings, reorder texts, and acknowledgments of user updates.

**Existing dependencies:** An off-the-shelf container, a printed lid, ESP32, ToF sensor, temperature/humidity sensor, accelerometer, battery, and e-ink display. The container temporarily hosts setup Wi-Fi, then joins home Wi-Fi and sends readings to the server through POST requests.

The hardware MVP is ready, while battery life is still being improved. Do not assume a ToF reading directly measures bean weight or that every cup uses the same quantity. “Ordered” is user-reported status, not verified retailer data. The product owner explicitly prefers fewer reminders over aggressive reminders based on uncertain estimates.

## Timeline

**Target: October 1, 2026.** Launch includes the working hardware MVP, phone setup, server-side readings and estimates, lid information, text configuration, two-stage reorder reminders, “ordered” handling, automatic refill detection, storage warnings, and a low-battery warning.

Calibration, alert thresholds, texting setup, and battery-related reporting intervals need resolution before launch. Affiliate revenue and live pricing are deferred.

## Appendix

### One-sentence product descriptions

- t-r1 is a smart coffee container that predicts when you’ll run out of beans and texts you a link to reorder.
- t-r1 helps home coffee drinkers stay stocked by tracking their beans and reminding them to reorder in time for delivery.

### Example reorder text

> You’re running low on coffee.  
> About 4 days remaining.  
> Lavazza Super Crema  
> Reorder: [saved product link]  
> Reply ORDERED to stop reorder reminders until your next refill.

### Reminder example

| Typical delivery time | First reminder | Follow-up if not acknowledged |
|---|---|---|
| 3 days | About 4 days remaining | About 3 days remaining |

### Validation notes

Record accepted readings, estimates, detected refills, reminder events, and “ordered” replies. Compare these with household feedback and observed refill events. Reminder timing alone does not prove that depletion predictions are accurate.
