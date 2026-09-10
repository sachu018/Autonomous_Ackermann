# Scratchpad — Mistakes, Root Causes, and Corrections

This document tracks mistakes encountered during the project evolution, how they were diagnosed, and how they were corrected.

---

### Mistake 1: Missing `config.h` Include in `steer_pid.c`
- **What happened:** `SteerPID_Update()` in `steer_pid.c` used `LOOP_HZ` for fallback $dt$ calculation, but never included `config.h`.
- **How it was found:** Code audit of `#include` dependencies.
- **How it was corrected:** Added `#include "config.h"` to `steer_pid.c`.

---

### Mistake 2: Kinematic Turning Radius Calculated with `sinf()` instead of `tanf()`
- **What happened:** `Ackermann_ComputeRPM()` used `R_c = ACK_WHEELBASE_M / sinf(delta_rad)` to calculate turning radius. In bicycle kinematics, turning radius about the rear axle center is $R_{\text{rear}} = \frac{W_b}{\tan(\delta)}$. Using $\sin(\delta)$ computed radius to the front axle ($R_{\text{front}} = \frac{W_b}{\sin(\delta)}$), causing a ~41% error at $45^\circ$ lock ($0.80\text{ m}$ true vs $1.13\text{ m}$ computed) and severe tire scrub.
- **How it was found:** Mathematical kinematic derivation from the Ackermann right triangle.
- **How it was corrected:** Replaced `sinf(delta_rad)` with `tanf(delta_rad)` in `Ackermann_ComputeRPM()`.

---

### Mistake 3: Actuator Stalled on Single-Sample ADC Glitch (Bypassing 5-Tick Debounce)
- **What happened:** `ads1115.c` had a 5-tick debounce threshold (`STEER_FAULT_THRESHOLD`), but set `out->valid = 0` on every single failed read. The main loop checked `steer.valid` directly, causing a single dropped I2C transaction to immediately freeze the actuator and reset the PID integral.
- **How it was found:** Logic tracing of `out->valid` assignment vs `main.c` actuator control gate.
- **How it was corrected:** Retained the previous valid steering angles during transient faults and kept `valid = 1` until `fault_runs >= STEER_FAULT_THRESHOLD`.

---

### Mistake 4: Reverse Differential Ratio Distortion from Post-Hoc Clamping
- **What happened:** `Ackermann_ComputeRPM()` properly rescaled both wheel RPMs together when either exceeded `WHEEL_RPM_MAX` (20 RPM), but `_ToThrottle()` independently clamped reverse RPM to `THR_REV_RPM_CAP` (10 RPM). On a sharp reverse turn (e.g. inner wheel 6 RPM, outer wheel 12 RPM), only the outer wheel was clamped, compressing the 2:1 differential ratio down to 1.66:1.
- **How it was found:** Tracing RPM limits and throttle conversion across forward vs reverse modes.
- **How it was corrected:** Passed the active direction/RPM limit into `Ackermann_ComputeRPM()` to scale both wheels down proportionally before throttle generation.

---

### Mistake 5: Hardcoded Magic Number `0.183f` for Reverse Max Velocity
- **What happened:** `v_max_dir` used a bare float literal `0.183f` in `ackermann.c`, which represented $(10\text{ RPM}/60) \times 2\pi R$, decoupling it from `THR_REV_RPM_CAP`.
- **How it was found:** Code review of constant propagation.
- **How it was corrected:** Defined `VMAX_REV_MS` derived explicitly from `THR_REV_RPM_CAP` and `WHEEL_RADIUS_M`.

---

### Mistake 6: Actuator Duty Cycle Shaved to 99.9%
- **What happened:** With `ACT_PWM_ARR = 999`, full 100% duty cycle in STM32 PWM Mode 1 requires `CCR = 1000` (`ARR + 1`). Clamping CCR to `ACT_PWM_ARR` (999) limited output to 99.9%.
- **How it was found:** STM32 Timer PWM compare register behavior analysis.
- **How it was corrected:** Adjusted maximum compare register value to `ACT_PWM_ARR + 1` (1000) when duty cycle is 100%.

---

### Mistake 7: Derivative Kick on Setpoint Changes in Steering PID
- **What happened:** `SteerPID_Update()` calculated derivative on error: $\frac{d(e)}{dt} = \frac{d(\text{target} - \text{actual})}{dt}$. When the transmitter stick was moved quickly, the step-change in `target_deg` caused a massive derivative spike ("derivative kick"), lurching the actuator.
- **How it was found:** Control systems analysis of setpoint step response.
- **How it was corrected:** Switched to **Derivative-on-Measurement**: $\text{derivative} = -\frac{\delta_{\text{actual}} - \delta_{\text{prev\_actual}}}{dt}$.

---

### Note: Potentiometer Raw ADC Calibration Done Off-Board (ESP32 rig)
- **What happened:** Rather than flashing `Rover_closed_loop` and calibrating through the STM32's own `ADS1115_ReadRaw()` + existing PA9→ESP32 debug telemetry link, the raw min/max was captured with a standalone ESP32 + `Adafruit_ADS1X15` sketch wired identically to the final install (same `0x48` address, same AIN0=left/AIN1=right assignment, same 3.3V pot excitation, and — critically — gain forced to `GAIN_ONE` (±4.096V) to match the STM32 firmware's `PGA=001` config word, since the raw ADC code is gain-dependent and the library's default gain, `GAIN_TWOTHIRDS` (±6.144V), would have produced non-portable numbers).
- **Why this is valid:** the ADS1115's raw output is a function only of the analog voltage on the pin and the PGA gain setting — not of which MCU reads it — so with gain and wiring matched, the numbers transfer directly to `ackermann_config.h` without touching the STM32 firmware.
- **Measured raw values (lock-to-lock sweep):**

  | | Left pot (`AIN0`) | Right pot (`AIN1`) |
  |---|---|---|
  | Extreme **left** turn (MAX_RAW) | 15766 | 14279 |
  | Extreme **right** turn (MIN_RAW) | 6813 | 6059 |

  Both channels move the same direction for the same turn (higher raw = left lock), consistent with the sign convention documented in `ackermann_config.h`. Both well inside the `ADC_SANITY_MIN/MAX` (200–32000) guard band. Span differs L=8953 vs R=8220 (~9%), attributed to normal pot-to-pot tolerance — each channel is calibrated independently so this doesn't affect correctness.
- **⚠️ Open item:** the actual mechanical lock-to-lock angle was **not** measured with a protractor — `MAX_STEER_ANGLE_DEG` (45.0°) in `ackermann_config.h` is still the assumed design value, unverified against hardware. If the true lock angle differs, the linear raw→angle mapping will be off even though the raw calibration itself is correct (see architecture.md Phase 4 Step 2).
- **Applied:** `ADC_L_MIN_RAW=6813`, `ADC_L_MAX_RAW=15766`, `ADC_R_MIN_RAW=6059`, `ADC_R_MAX_RAW=14279` in `ackermann_config.h`.

---

### Note: Steering Deadband Widened, Twice — 0.5° → 1.3° → 2.5°
- **What happened:** `STEER_DEADBAND_DEG` first changed `0.5f` → `1.3f` pre-emptively (not yet hardware-tested). After flashing and testing on real hardware, the actuator was confirmed **hunting/oscillating trying to settle at zero** — consistent with the predicted cause: the actuator is 100%-speed bang-bang (no low-speed throttling, see architecture.md §2.1) so it overshoots the deadband window each correction, flips direction, overshoots the other way, repeats.
- **Correction:** widened further to `2.5f` to give enough margin for the actuator's overshoot at full speed / 20 Hz loop reaction time.
- **Applied:** `STEER_DEADBAND_DEG = 2.5f` in `ackermann_config.h`. ⚠️ Not yet re-verified on hardware whether 2.5° fully eliminates the hunting, or whether it needs to go wider still (or whether the real fix is eventually throttling actuator speed near the setpoint instead of widening the band indefinitely).

---

### Note: Right-Side Zero Offset Traced to Per-Wheel Calibration Disagreement, Not Just an Average Bias
- **What happened:** After the deadband fix, wheels still settled offset to the right at commanded straight (Xn=0). Investigated by having the user physically point the wheels dead straight and read `raw_L`/`raw_R` at that exact position: `raw_L=13098, raw_R=10315`.
- **Diagnosis:** Running these through the existing (untrimmed) `_MapToAngle()` calibration (`ADC_L_MIN/MAX_RAW=6813/15766`, `ADC_R_MIN/MAX_RAW=6059/14279`) gave `angle_L=+18.18°`, `angle_R=+1.60°` — average `delta=+9.89°` instead of 0°. The **average** offset alone would be explainable by the known limitation that the linear 2-point calibration assumes both mechanical locks sit at exactly ±`MAX_STEER_ANGLE_DEG` (45°) from true center (see the earlier calibration note above) — a small asymmetry there produces a small average bias. But an **18.18° vs 1.60° disagreement between the two wheels at the same physical state** is far larger than that alone explains, and meant the electronic differential (which treats `delta` as if both wheels are symmetric about it) was silently trusting a `delta` that didn't represent either wheel accurately — not just an average-zero problem.
- **Root cause (best current explanation, not fully isolated):** most consistent with either (a) the left pot/linkage having drifted or slipped since the original lock-to-lock raw calibration, or (b) inherent left/right steering-arm geometry asymmetry beyond what a simple ±45°-symmetric 2-point model can represent. **Not independently confirmed on hardware** (e.g. re-checking that `raw_L` still hits ~15766/~6813 at the mechanical locks) — flagged to the user as worth checking, but user opted to proceed directly to a software trim rather than re-verify mechanically first.
- **How it was corrected:** added independent per-wheel trims, `ANGLE_TRIM_L_DEG=18.18f` and `ANGLE_TRIM_R_DEG=1.60f`, subtracted inside `_MapToAngle()` (now takes a `trim_deg` parameter) in `ads1115.c`, applied per-channel in `ADS1115_Read()`. Verified in isolation (Python re-implementation of the exact formula) that at `raw_L=13098`/`raw_R=10315`, both `angle_L` and `angle_R` now independently compute to ~0.00°, not just their average.
- **⚠️ Open item:** these trims are only valid for the current `ADC_*_MIN/MAX_RAW` calibration. If the pot calibration is ever redone (re-swept lock-to-lock), the straight-ahead raw values must be re-measured and these two trims recalculated — they do not automatically adapt. Also not yet re-verified on actual hardware after flashing (only verified analytically).

---

### Note: Pivot Mode Changed From True Stationary-Inner Pivot to Tight Crawl-Turn
- **What happened:** `Ackermann_Run()`'s standstill pivot previously held the inner rear wheel at exactly 0 RPM and drove only the outer wheel forward at a fixed crawl speed derived from `PIVOT_CRAWL_MS` (~7.5 RPM), once the front wheels reached `PIVOT_LOCK_FRACTION` (80%) of max lock.
- **Changed (user request):** both wheels now roll forward during pivot — outer at `PIVOT_OUTER_RPM=10.0`, inner at `PIVOT_INNER_RPM=6.0` — replacing the old `PIVOT_CRAWL_MS`-derived single crawl speed. This is no longer a true stationary-inner-wheel pivot; it's a tight differential crawl-turn.
- **Preserved unchanged:** the `lock_achieved` gate (front wheels must reach ≥`PIVOT_LOCK_FRACTION` of `MAX_STEER_ANGLE_DEG` before either wheel is driven) — confirmed with the user this gating behavior should stay as-is.
- **Applied:** `ackermann_config.h` (`PIVOT_OUTER_RPM`, `PIVOT_INNER_RPM` replacing `PIVOT_CRAWL_MS`), `ackermann.c` (`Ackermann_Run()` pivot branch).

---

### Mistake 8: Standstill Steering Snapped to Full Lock Instead of Proportional to Stick
- **What happened:** User confirmed (twice, before and after hardware testing) that steering angle should be linearly proportional to `Xn` across the whole FS-i6X stick range, with the actuator always driving at 100% speed toward whatever target that produces (already true in the throttle-applied branch of `Ackermann_Run()`, verified against `ibus.c: _ToFloat()` which normalizes the raw iBUS PWM linearly with no expo curve). On hardware testing steering alone (throttle stick neutral), holding Xn at ~25-30% drove the actuator all the way to full lock instead of holding ~25-30% of `MAX_STEER_ANGLE_DEG`.
- **How it was found:** Re-reading `Ackermann_Run()`'s standstill branch (`abs_y < ACK_STICK_DEADBAND && abs_x > ACK_STICK_DEADBAND` — i.e. exactly "steering stick moved, throttle neutral", which is what the user was testing): it hardcoded `target_steer_deg = (Xn<0) ? +MAX_STEER_ANGLE_DEG : -MAX_STEER_ANGLE_DEG` — snapping to full lock the instant the stick cleared its own 5% deadband, regardless of magnitude. The proportional formula (`-Xn * MAX_STEER_ANGLE_DEG`) only existed in the separate throttle-applied branch, which the user's steering-only test never entered.
- **How it was corrected:** Changed the standstill branch to use the same proportional formula as normal driving. The pivot-crawl drive logic (`lock_achieved` gate at 80% of max lock) is unaffected and still only fires when the *measured* angle gets that far — which now only happens if the stick is genuinely held near its own extreme, since the target itself no longer artificially forces the wheels there for a moderate stick input.

---

### Note: Left Potentiometer Replaced — Full Re-Calibration
- **What happened:** User replaced the left steering potentiometer with a new unit and re-ran the ESP32 raw-value sweep (same rig/method as the original calibration). New readings: extreme left `raw_L=4107, raw_R=12500`; extreme right `raw_L=1920, raw_R=5722`; center (straight) `raw_L=3500, raw_R=10000`.
- **Note:** only the left pot was reportedly replaced, but *both* channels' raw ranges shifted from the original calibration (right was previously 6059–14279, now 5722–12500) — plausible if the shared steering linkage was disturbed during the swap, but not independently confirmed.
- **⚠️ Flagged, not resolved:** the new left pot's span is `4107−1920=2187` counts vs. the right's `12500−5722=6778` — a **3.1x** difference, much larger than the ~9% span mismatch on the original pair. This could mean: a different-value pot than intended, the wiper not using its full mechanical rotation over the ±45° swing, or a wiring/mounting issue. Not blocking — user asked to proceed with the calibration update — but worth checking the new pot's datasheet value and wiring before trusting it long-term. Left channel now also has ~3x coarser angular resolution per ADC count than the right, more exposed to noise.
- **Applied:** `ackermann_config.h` — `ADC_L_MIN_RAW=1920, ADC_L_MAX_RAW=4107, ADC_R_MIN_RAW=5722, ADC_R_MAX_RAW=12500`. Per-wheel zero trim re-derived the same way as before (compute `_MapToAngle()` at the center raw pair against the new min/max, that value becomes the trim to subtract): `ANGLE_TRIM_L_DEG=20.02f`, `ANGLE_TRIM_R_DEG=11.80f`. Verified analytically (Python re-implementation) that at `raw_L=3500`/`raw_R=10000`, both `angle_L` and `angle_R` now compute to ~0.00°. Not yet re-verified on flashed hardware.
- **⚠️ This trim design turned out to be flawed — see Mistake 9 below.** Superseded by a 3-point piecewise-linear calibration; `ANGLE_TRIM_L/R_DEG` no longer exist.

---

### Mistake 9: Flat-Offset Zero Trim Compressed One Side's Range, Broke Left Pivot
- **What happened:** User reported left pivot never triggering even at full left lock, while right pivot worked correctly.
- **How it was found:** Traced the per-wheel trim design (Mistake/Note above): `_MapToAngle()` computed `angle = -45 + norm*90 - trim_deg`, THEN clamped to ±45°. Subtracting a flat degree offset before clamping shifts the *entire* line, not just the zero point. With `ANGLE_TRIM_L_DEG=20.02`, at true full left lock (raw=`ADC_L_MAX_RAW`, norm=1) the raw computation gives `45 - 20.02 = 24.98°` for the left wheel (not 45°); combined with the right wheel's own trim-shifted `33.2°`, `delta = 29.09°` — short of the `36°` (`PIVOT_LOCK_FRACTION × MAX_STEER_ANGLE_DEG`) needed to trigger the pivot. The right side happened to hit the -45° clamp early instead (saturating before the true right-lock raw value), which is why right pivot "worked" — but was also technically wrong, just in a way that was invisible (saturating early looks the same as reaching the true limit).
- **Root cause:** a flat-degree trim applied to a single min→max line is mathematically the wrong tool for fixing a zero-point — it necessarily unbalances the ±45° range around the new zero, since the two locks are not equidistant from the corrected center in raw terms.
- **How it was corrected:** replaced with a proper 3-point (piecewise-linear) calibration. Added `ADC_L_CENTER_RAW`/`ADC_R_CENTER_RAW` (3500/10000, the same straight-ahead raw values used to derive the old trims) to `ackermann_config.h`. Rewrote `_MapToAngle()` in `ads1115.c` to take `(min_raw, center_raw, max_raw)`: raw ≥ center maps linearly to `0..+45°` against `(center, max)`; raw < center maps linearly to `-45..0°` against `(min, center)`. This guarantees center reads exactly 0° and *each* lock reaches exactly ±45° independently, regardless of how far either lock's raw value is from center — no compression, no early saturation. Removed `ANGLE_TRIM_L_DEG`/`ANGLE_TRIM_R_DEG` entirely (superseded, not just unused).
- **Verified analytically** (Python re-implementation of the exact formula): straight → `0.0°/0.0°` (unchanged from before), full left lock → `+45.0°/+45.0°` (`delta=45.0`, was `29.09`), full right lock → `-45.0°/-45.0°`. **Not yet re-tested on flashed hardware.**
- **Lesson for later calibration work:** if the pots are ever replaced or re-swept again, this design needs all **three** raw points (min, center, max) re-measured together, not just min/max — the center point is now load-bearing in the mapping itself, not an afterthought correction.

---

### Mistake 10: Left Drift on "Straight" — Two Different "Close Enough to Straight" Thresholds Disagreed
- **What happened:** User reported the rover consistently drifting left whenever commanding straight ahead, requiring a small manual right stick correction to actually track straight.
- **How it was found:** Traced the two independent thresholds that each separately decide "is this straight enough":
  - `main.c` step 6 (actuator control): `STEER_DEADBAND_DEG = 2.5°` — the actuator stops correcting once `|target - actual| <= 2.5°`. So on a straight command (`target=0`), the actuator can legitimately rest anywhere in `[-2.5°, +2.5°]`.
  - `ackermann.c: Ackermann_ComputeRPM()`: `ACK_STRAIGHT_DEG = 1.0°` — only below this does it force `V_left = V_right` exactly; above it, it computes a real `tan(delta)`-based L/R differential (by design: `delta > 0` → right wheel outer → faster, which curves the rover left).
  - **The gap:** any residual angle between 1.0° and 2.5° was inside the actuator's "good enough, stop moving" zone but outside the differential's "treat as straight" zone — so the actuator would settle there and stop, while the rear differential kept actively steering based on that small residual. A systematic tendency for the actuator's bang-bang control to rest on the left side of that window (plausible from asymmetric overshoot/approach dynamics, not independently proven) would produce a *consistent* left drift, matching the reported symptom exactly.
- **How it was corrected:** Raised `ACK_STRAIGHT_DEG` from `1.0f` to `3.0f` (now `>= STEER_DEADBAND_DEG`), so nothing inside the actuator's acceptable resting zone can trigger a differential — the whole zone the actuator is willing to stop in is now also treated as exactly straight by the rear drive.
- **⚠️ Not yet re-tested on hardware.** If the rover still drifts after this fix, the steering-loop explanation is ruled out and the next suspect is asymmetry on the rear drive side itself (motor/DAC/controller differences between left and right, since the drive is open-loop DAC control with no per-wheel speed feedback) — worth testing by holding/blocking the front wheels dead straight (or disabling the actuator) and checking whether pure differential-drive still veers.
- **General rule going forward:** any threshold that decides "close enough" in one control loop (actuator position) must be reconciled against thresholds that consume its output in another loop (rear differential) — a narrower downstream threshold than the upstream deadband recreates this exact class of bug.

---

### Note: SWB Repurposed From Reverse-Speed-Cap to Manual/Auto Select (Design Decision, Pre-Implementation)
- **What happened:** Planning autonomous operation via an RPi companion computer over UART (PA2/PA3). Needed a physical switch on the FS-i6X transmitter to arbitrate between RC control and RPi-issued commands.
- **Decision:** Repurpose `SWB` (currently the 2-position reverse-speed-cap switch, `SWB_SPEED_LOW/HIGH` = 60%/100% in `config.h`/`ibus.c`) as the MANUAL/AUTO command-source select, per explicit user choice over the alternatives (moving reverse-cap onto SWC, or finding a free channel instead).
- **Consequence:** the reverse-speed-cap feature is dropped entirely — once implemented, reverse will run at the same cap as forward (no separate limiter). This is a deliberate trade, not an oversight.
- **Not yet implemented.** No firmware change has been made yet — this is recorded ahead of implementation so the "why" isn't lost. See architecture.md §3.3.

---

### Note: UART Link-Loss Failsafe for Autonomous Mode — Brake, Same Class as RC Loss
- **What happened:** Needed to decide STM32 behavior if the RPi crashes or the UART link drops while SWB is in AUTO.
- **Decision (explicit user choice over "hold last command" and "silently fall back to RC"):** treat it exactly like the existing 400 ms RC-loss failsafe — a new `AUTO_UART_TIMEOUT_MS` (~300 ms) with no valid command frame → force `BRAKE`, regardless of the last command received or of SWB's position. Flipping SWB to MANUAL always hands control back to the RC sticks immediately, independent of UART link state.
- **Rationale:** "hold last command" risks the rover driving blind on an RPi hang; "silent fallback to RC" requires an operator to be actively shadowing the sticks at all times, which isn't a safe assumption to build in as the default.
- **Not yet implemented.** See architecture.md §3.4–3.5.

---

### Note: STM32↔RPi UART Link — RESOLVED (Bluetooth Hijack + Missing `enable_uart=1`)
- **What happened:** Continuing from the code audit below — user confirmed via the ESP32 sniffer that the STM32 was transmitting correctly the whole time, and independently re-confirmed physical wiring was correct. This fully isolated the fault to the RPi's OS/config.
- **First finding:** `dmesg` showed `hci_uart_bcm serial0-0: ...` — the Pi 5's onboard Bluetooth had attached to the same UART as GPIO14/15, and `stty` showed the port stuck at 9600 baud (BT's initial handshake rate) instead of 115200. Fixed with `dtoverlay=disable-bt` in `config.txt` (permanently disables BT on this Pi — accepted, not used elsewhere in this project). Confirmed via `dmesg` post-reboot: no more `hci_uart_bcm` attach.
- **Bluetooth fix alone wasn't enough** — raw byte capture still showed zero bytes even with baud explicitly forced to 115200 via `stty` (note: plain `cat /dev/serial0` does NOT itself set the baud rate — it inherits whatever the port is currently configured to, which caused a confusing red herring earlier where the "raw" test looked like it disagreed with itself).
- **Real root cause found via `pinctrl get 14,15`** (RP1's pin-mux inspection tool — had to `apt install` it since `raspi-gpio`, the older tool, doesn't exist as a package for the Pi 5's RP1 chip and isn't in this OS's repos anyway): showed `GPIO14 = none`, `GPIO15 = none`. The physical pins were **never actually switched into UART alternate-function mode**, despite `/dev/ttyAMA10` existing as a device node and (earlier in this project) a `serial-getty` unit having been "active" on it. **Lesson: neither of those prove the GPIO pins are hardware-muxed to the peripheral** — that was the wrong signal trusted throughout the original §3.6 setup, and cost significant debugging time here. The actual fix — `enable_uart=1` in `config.txt` — was never added in the original setup because the console appearing to work was (wrongly) taken as proof the UART hardware was fully configured.
- **Verified fixed:** post-reboot, `pinctrl get 14,15` → `GPIO14 = TXD0`, `GPIO15 = RXD0`. Raw capture showed clean, correctly-checksummed frames (hand-verified one by XOR). `check_stm32_link.py` → steady 20.0 frames/s, 0 bad, `ARMED=Y RC_OK=Y`. `read_encoders.py` → sane live data.
- **New observation, not investigated yet:** `angle_R` reads a perfectly steady `-45.00°` with zero jitter, while `angle_L` shows normal small ADC noise (~±0.1-0.2°) around `+39°`. Could be genuine (right wheel actually at full lock on the jack) or a saturating/clamped channel. `STEER_FAULT` bit is *not* set, so not flagged as a hard fault by the STM32 — but worth a physical look next time someone's at the rover, and cross-checking against the known ⚠️ open item in architecture.md Phase 4 Step 2/3 about unverified true lock angle and the 3.1x pot-span mismatch (scratchpad's "Left Potentiometer Replaced" note) — this could be related to either of those pre-existing open items rather than something new.

---

### Note: STM32↔RPi UART Link — Firmware Code-Audited, Still Unresolved; ESP32 Sniffer Built
- **What happened:** User confirmed STM32 is powered and (per them) correctly wired, and that the contactor clicks on boot — ruling out a boot-time hang. Did a full code audit of the firmware path relevant to the zero-bytes symptom (see architecture.md §3.8 for the table of what was checked and ruled out): confirmed `RPiLink_SendFeedback()` is called unconditionally every tick in all modes, confirmed the flashed `.elf` really contains the Phase 5 code (timestamp + symbol check), confirmed no GPIO conflict on PA2/PA3, confirmed DMA1's clock is enabled (so `MX_USART2_UART_Init()` shouldn't be failing into `Error_Handler()`), confirmed no unbounded I2C call could be hanging the boot path.
- **Could not find a firmware bug by static review.** The contactor-click fact is the strongest evidence the main loop is actually running and reaching the `RPiLink_SendFeedback()` call — which shifts suspicion toward something in the physical layer (PA2 pin itself, a connector/solder issue, or something about that specific net) that static code reading can't resolve.
- **Built `ESP32_uart_sniffer/ESP32_uart_sniffer.ino`** — a read-only tap on STM32 `PA2` only (deliberately one-way, same safety rule as the existing PA9 debug bridge: never wire its TX to `PA3` while the RPi is also on that line, to avoid two transmitters driving one wire). This isolates the STM32-to-tap-point leg from the RPi entirely — a definitive split test.
- **Not yet run** — user is about to wire it up. Next message should report what the sniffer shows.

---

### Note: STM32↔RPi UART Link — Zero Bytes Received, Cause Not Yet Isolated
- **What happened:** Built `RPi_companion/uart_link.py` (shared protocol driver), `check_stm32_link.py`, and `read_encoders.py`. Deployed and ran `check_stm32_link.py` live against the physically-connected setup (user confirmed STM32+RPi UART wired, GPIO14/15 per the earlier pin note).
- **Result:** zero valid feedback frames received in 8s. Bypassed all app-level parsing and did a raw byte capture directly on `/dev/serial0` (`cat /dev/serial0 | od -A x -t x1z`) for 5s — **zero raw bytes**, confirming this is not a bug in the new parsing code (checksum/framing logic never even got a chance to run) but something upstream: no data is reaching the Pi's RX pin at all. `stty -F /dev/serial0` confirms the port itself is correctly configured at 115200 baud.
- **Not yet isolated — needs physical inspection by the user, cannot be diagnosed further remotely:** candidates are (a) STM32 not currently powered, (b) TX/RX wiring wrong (not crossed, or wrong physical pin — position vs GPIO-number mixups are an easy mistake on the 40-pin header), (c) common GND not connected between the two boards, (d) STM32 not actually running the Phase 5 firmware (less likely, but not ruled out).
- **Next step:** user to check power + wiring, report back; then re-run the raw capture first (fastest signal — either bytes start arriving or they don't) before re-running `check_stm32_link.py`.

---

### Note: RPi5 OS-Level Setup Completed Remotely (I2C, UART Console, Python Env)
- **What happened:** SSH access was set up (key-based, replacing the initial password login) and used to complete the RPi5 OS-level prerequisites from architecture.md §3.6, with the Pi on the bench (no IMU or STM32 physically attached).
- **Environment specifics found, worth knowing for later work on this Pi:** Debian 13 "trixie" (not Bookworm — no `raspi-config` present, so all changes were made by editing `/boot/firmware/config.txt`/`cmdline.txt` and `systemctl` directly instead). The Pi 5's UART on GPIO14/15 shows up as `ttyAMA10` (not `ttyAMA0` as on older Pis) — `/dev/serial0` symlinks to it as expected, but if referencing the device node directly anywhere, use `ttyAMA10` for this specific unit, or better, always go through the `/dev/serial0` alias since the exact number could differ Pi-to-Pi depending on what else RP1 exposes.
- **I2C1 (header pins) was not enabled by default** — `dtparam=i2c_arm=on` was present in `config.txt` but commented out. Two *other* I2C buses (`i2c-13`, `i2c-14`, RP1-internal) already existed and could easily be mistaken for the header bus if not careful — the header bus is specifically `i2c-1` (confirmed via `i2cdetect -l` showing "Synopsys DesignWare I2C adapter" after enabling and rebooting).
- **UART console removal needed two changes, not one:** disabling `serial-getty@ttyAMA10.service` alone would not have survived a reboot on its own, because that unit is generated at boot time from the `console=serial0,115200` kernel parameter in `cmdline.txt` — removing that parameter is what actually makes the getty stay gone permanently; `systemctl disable --now` only handled the *current* boot's instance.
- **`pip install -r requirements.txt` failed twice before succeeding**, both times because of missing system (not Python) packages needed to compile the `lgpio` backend from source: first `swig` (no `swig` binary → can't generate the C wrapper), then `liblgpio-dev` (the runtime `.so` was present but not the dev symlink `ld` needs to link `-llgpio`). Both installed via `apt-get`; `requirements.txt` now documents this upfront so a future setup doesn't have to rediscover it by trial.
- **Verified as far as possible without hardware:** `board.I2C()` correctly identifies `RASPBERRY_PI_5` (`SCL=GPIO3`, `SDA=GPIO2`, matching the wiring), `i2cdetect -y 1` shows a clean empty bus (no false positives), and `test_imu.py` fails with a clear `No I2C device at address: 0x28` + exit code 1 rather than hanging or crashing. This confirms the whole software stack up to the point where real hardware is required.
- **Not yet done:** anything requiring the physical BNO055 or the STM32 UART link — both need the Pi reinstalled in the rover with peripherals reconnected.

---

### Mistake 12: Lookahead Search Skipped Segment 0, Found Only By Running On Real Hardware
- **What happened:** User asked for automatic path/log archival and a much richer telemetry log (cross-track error, Pure Pursuit internals, full raw IMU) for tuning. While adding those, ran the very first real-hardware test of `mission.py` (IMU + STM32 link both real, not stubbed) — the logged `lookahead_x=10.0` (the path's FINAL waypoint) instead of the expected `~0.8` (the lookahead distance) for a rover sitting stationary at the path's start.
- **Root cause:** `_advance_nearest_index()` advances the tracking cursor past waypoint 0 immediately, because every real mission starts with the rover sitting exactly ON waypoint 0 (`pose=(0,0)` is *defined* to equal `waypoints[0]` at mission start — distance 0, always under the 0.25m threshold). `_find_lookahead_point()`'s segment search then started exactly AT the advanced cursor (index 1), skipping segment 0→1 — precisely the segment that actually intersects the lookahead circle. Every later segment genuinely lies outside the 0.8m circle from the origin, so the search fell through all of them to its "whole path closer than Ld" fallback: aim at the final waypoint.
- **Why the smoke test didn't catch it:** `guidance.py`'s `__main__` test started the synthetic rover 0.5m off-axis — never landing exactly on waypoint 0, so `_advance_nearest_index()` never advanced on the first tick, and the bug's precondition never triggered. A real mission's actual starting condition (exactly on waypoint 0) is a case the smoke test simply didn't represent.
- **How it was corrected:** both `_find_lookahead_point()` and `_cross_track_error()` now start their segment search one index behind the cursor (`max(0, nearest_idx - 1)`), not at it — standard handling for this well-known Pure Pursuit edge case. Added Check 2 to `guidance.py`'s smoke test, specifically starting the rover exactly on waypoint 0 (matching the real-mission case) and asserting the lookahead point is near, not at the path's end — this check would have caught the bug before hardware did.
- **Lesson, worth generalizing:** a smoke test's synthetic starting conditions should match the REAL caller's actual starting conditions, not just "some plausible input" — the 0.5m-off-axis start was a reasonable test of steering convergence, but it accidentally avoided the exact state (`pose == waypoints[0]`) every real run begins in. This is the second bug this session alone that only surfaced by actually running on real hardware rather than trusting synthetic tests (see Mistake 11) — reinforces that synthetic smoke tests catch *some* classes of bug, not all, and real-hardware runs remain necessary even after they pass.
- **Verified fixed:** re-ran the strengthened smoke test (lookahead now `(0.80, 0.00)`, correct) and re-ran `mission.py` on the real Pi — confirmed via the live printed line and the telemetry log.

---

### Note: mission.py Now Auto-Archives Paths + Much Richer Telemetry Logging
- **What happened:** User asked for (1) generated/loaded paths to always be saved, not just optionally via `--save-csv`, and (2) a comprehensive log with all sensor/telemetry data for plotting and parameter tuning.
- **Path archival:** every run now saves the path actually used (generated OR loaded via `--csv`) to `logs/mission_<timestamp>_path.csv`, sharing the exact same timestamp as that run's telemetry log — trivially paired for plotting later. `--save-csv` still exists separately, for saving a copy somewhere reusable (e.g. to `--csv` back in on a later run) — that's now clearly optional/additional, not how the run's own path gets recovered.
- **Telemetry log expanded from 15 to 31 columns.** Added: `cte_m` (signed cross-track error — the actual metric the old BBB/RTK systems' gains were tuned against, e.g. `ismc_controller.py`'s `ey`), `alpha_deg` (heading error to the lookahead point), `lookahead_x/y_m`, `target_idx`, `mission_finished` (all from `guidance.PurePursuit`, exposed as `last_*` instance attributes populated each `update()` call); and the full raw IMU — `roll_deg`, `pitch_deg`, `gyro_z_rads`, `accel_x/y_ms2`, all four calibration fields, `valid` (previously only `heading_deg` was indirectly used, nothing else from `IMUData` was logged at all).
- **Implementation note:** `Odometry` now exposes `last_imu_data` (the full `IMUData` from its most recent `update()` call) so `mission.py` doesn't need a second, redundant `imu.read()` call per tick (which would have doubled real I2C traffic for no reason — `imu.py`'s `read()` performs an actual transaction every call, it isn't cached).
- **Verified on real hardware:** ran on the RPi with real IMU + STM32 link (first real-hardware run of `mission.py` at all) — this is what surfaced Mistake 12 above.

---

### Mistake 11: Heading Reference Was Re-Locking on Every Mission Restart, Defeating the Point of "Compass-Referenced"
- **What happened:** User asked a clarifying question about the (0,0)/heading semantics — asking it surfaced that `Odometry.reset()` (called by `mission.py` on every SWB MANUAL→AUTO rising edge) set `self._yaw_ref = None`, which makes `update()` re-capture whatever the IMU currently reads as the NEW zero-heading on every single mission start/restart.
- **Why this was wrong:** the whole reason "compass-referenced" was chosen over "start-relative" (see the design discussion in architecture.md §3.9) was so a pattern keeps the same real-world orientation across repeated test runs in one session — e.g. running "straight line 10m" three times in a row should point the same real-world direction each time. Re-locking the heading reference on every restart made it functionally identical to "start-relative" (the option deliberately NOT chosen), since each restart got its own arbitrary +X direction based on whatever way the rover happened to be pointed at that exact moment.
- **How it was found:** not caught by the standalone smoke test at the time it was written — that test only exercised a single continuous run, never called `reset()` mid-test with a changed heading, so it couldn't have caught this. Found only because the user asked to clarify the (0,0)-on-AUTO-engage behavior, prompting a re-read of the actual code against what was designed.
- **How it was corrected:** decoupled position reset from heading-reference locking. `reset()` now only re-zeros `(x, y)`; the heading reference locks exactly once, on the first `update()` call ever made on that `Odometry` instance (effectively: whichever way the rover is pointed when `mission.py` is *launched*, not each time SWB flips to AUTO). Strengthened the smoke test to actually catch a regression of this: Check 2 calls `reset()`, changes the fake IMU's heading by 90°, and asserts `yaw` reports the real turn (~-90°) rather than silently re-zeroing — this specific test would have caught the original bug.
- **Lesson:** a smoke test that only exercises the "happy path" of a single continuous run won't catch a state-reset bug that only manifests across multiple start/stop cycles — worth deliberately testing the *reset* behavior, not just the update behavior, for any stateful module like this one.
- **Not yet re-verified on real hardware** (same caveat as every other calibration/behavior note) — verified only via the strengthened synthetic smoke test.

---

### Note: Both Steering Potentiometers Replaced — Third Raw ADC Calibration
- **What happened:** User replaced both steering potentiometers (not just the left one, as in the previous replacement — see "Left Potentiometer Replaced" note below) and provided freshly-measured raw ADC values directly (not via the ESP32 rig this time, values just given in conversation): full right `raw_L=3096, raw_R=2124`; full left `raw_L=5793, raw_R=4160`; center `raw_L=4426, raw_R=3167`.
- **How it was found:** User-supplied, not independently measured or re-derived by re-running the ESP32 rig this session.
- **How it was corrected:** Updated `ADC_L/R_MIN_RAW`, `ADC_L/R_MAX_RAW`, `ADC_L/R_CENTER_RAW` in `ackermann_config.h` directly to the new values (all six constants — the existing 3-point piecewise-linear calibration scheme from the previous replacement, `_MapToAngle()` in `ads1115.c`, needed no code changes, only new inputs). Updated the adjacent comment block, which had gone stale twice over (still cited the *original* pre-any-replacement values' rationale in one place, and the previous replacement's 3.1x span-mismatch warning in another) — replaced with the new values' own span math.
- **This calibration is healthier than the last one:** L span split 1330/1367 (~3%), R span split 1043/993 (~5%) — both well-balanced, unlike the previous replacement's 3.1x L/R mismatch that was flagged as a concern and never resolved. Reasonable evidence the new pots are seated correctly and using their full mechanical rotation.
- **Explicit scope boundary respected:** only these six `#define` values (plus the directly-adjacent explanatory comment) were touched. `MAX_STEER_ANGLE_DEG` (45°) was deliberately left unchanged — user has not yet supplied a protractor-measured true mechanical lock angle, only the raw ADC readings at the (assumed-45°) mechanical stops. Nothing else in the STM32 firmware was touched, per explicit user instruction.
- **⚠️ Not yet rebuilt or reflashed** — this is a source change only. User needs to rebuild in STM32CubeIDE and reflash before these values take effect on hardware. Until then, the rover is still running the previous pots' calibration values in flash, which are now wrong for the new pots.
- **Also not yet re-verified on hardware after flashing** (same caveat as every prior calibration round) — analytically the 3-point piecewise map still guarantees center→0°, each lock→±45° exactly, given accurate raw inputs; whether these particular raw inputs are themselves accurate (wiring, gain, actual mechanical stop position) is unverified beyond the values the user reported.

---

### Note: mission.py Built — Runnable Entry Point, STM32 Firmware Explicitly Off-Limits
- **What happened:** User accepted the 1.6m lawnmower turn-spacing finding as-is. Confirmed `MAX_STEER_ANGLE_DEG=45°` (used to derive `MIN_TURN_RADIUS_M` in `rover_config.py`) is a **reference/design value in the STM32 firmware, not yet verified against the true mechanical lock angle** — a steering potentiometer is about to be replaced and recalibrated. **Explicit instruction: do not change anything in the STM32 programs right now** — user will recalibrate and report back new values later.
- Built `mission.py`, the actual runnable entry point (everything before this was library modules with no program tying them together). Safety design: always computes+sends commands at 20 Hz regardless of SWB (matches the "inert while MANUAL" property already designed into `rpi_link.c`/`main.c`), and treats SWB's MANUAL→AUTO transition as the mission-(re)start trigger (resets odometry, re-creates the `PurePursuit` instance) rather than script-launch time — lets the operator abort mid-mission by flipping to MANUAL and restart clean by flipping back to AUTO, without restarting the script.
- **Verified failing cleanly off-hardware** on this laptop: IMU stub-mode warning printed, then a clean `sys.exit(1)` when `/dev/serial0` doesn't exist — no crash, no traceback. Argument parsing + pattern generation + CSV save all exercised via `--pattern straight --length 10 --save-csv`.
- **Still blocked on deployment** — the RPi has stayed unreachable from this laptop across this entire piece of work (still a different subnet, `10.32.x.x` here vs the Pi's last-known `10.242.x.x`). Nothing here has run on real hardware yet. Added `RPi_companion/logs/` to `.gitignore` preemptively (per-run mission logs, regenerable, unlike `Old_files/`'s deliberately-kept historical logs) — flagged to the user rather than assumed silently, since they'd explicitly chosen to keep everything for `Old_files/`.

---

### Note: Guidance Stack Built — Pure Pursuit + Dead Reckoning, No RTK
- **What happened:** Before writing any code, studied `Old_files/dev_bak/`'s BBB-era encoder+IMU-only autonomous scripts in depth (`ugv_p_waypoint.py`, `ugv_pid_waypoint.py`, and the circle/eight/lawnmower/9-axis variants — confirmed they're all one shared skeleton, the "pattern" lives entirely in which CSV gets loaded, not in the script). Had an explicit design discussion with the user before building anything — coordinate frame, algorithm choice, and every tunable parameter were discussed and decided in conversation before `rover_config.py`/`waypoints.py`/`odometry.py`/`guidance.py` were written. Full reasoning is in `architecture.md` §3.9; this note covers what's *not* already there.
- **Deliberate algorithm change from the prior art, not a straight port:** the BBB system used heading-error P/PID → angular velocity, which fits differential drive but has no natural mapping onto Ackermann steering (no direct spin-rate actuator). Pure Pursuit was chosen instead specifically because its output *is* a steering angle — maps directly onto `uart_link.py`'s `send_command(steer_target_deg, speed_target_ms)` with no intermediate V,W→wheel-RPM conversion step the old code needed.
- **Real physical-feasibility bug caught before it could matter:** while implementing the lawnmower row-turn generator, discovered the requested 1.0m row spacing implies a 0.5m-radius U-turn, tighter than this chassis's actual 0.8m minimum turn radius (`WHEELBASE_M / tan(MAX_STEER_ANGLE_DEG)`). Rather than silently generating an undrivable path, `_semicircle_turn()` in `waypoints.py` detects this (compares against `MIN_TURN_RADIUS_M`) and clamps to the achievable 0.8m radius, printing a warning and landing the turn 1.6m over instead of 1.0m. **Flagged to the user, not yet resolved** — a true tight-1.0m turn would need a reverse/three-point-turn maneuver, deliberately not built without confirming it's worth the complexity.
- **Verified without hardware:** all three logic modules have a `if __name__ == "__main__"` smoke test using synthetic data (fake IMU for `odometry.py`, a crude kinematic step for `guidance.py`) — all three ran clean on this dev laptop before ever touching the Pi. `waypoints.py`'s self-test independently confirmed the lawnmower finding above by arithmetic (4 rows/3 turns → final offset exactly 4.8m = 3×1.6m, matching the clamped-radius prediction).
- **Not yet deployed to the RPi** — network connectivity from this laptop to the Pi was down (different subnet, `10.32.x.x` vs the Pi's last-known `10.242.x.x` — not the usual mDNS flakiness, an actual network change) at the time these were written. Deploy once reachable again.
- **Not yet built:** `mission.py` — the top-level loop that would actually call `uart_link.send_command()` using these modules. Nothing here drives the rover yet, by design (discussion-first, as the user asked).

---

### Note: RPi↔STM32 Interconnection Ported into Rover_study.md (New §14)
- **What happened:** User asked to fold the now-verified RPi↔STM32 interconnection into `Documentations/Rover_study.md` — the "complete technical reference for a new engineer" document, distinct from `architecture.md` (the working/planning doc this whole Phase 5 effort has been tracked in).
- **Important distinction preserved:** `Rover_study.md` describes the original **open-loop** `Rover/` build (`pipeline.c`/`kinematics.c`), not `Rover_closed_loop/` where the RPi link actually lives. Rather than blending the two, added a prominent note at the top of the document and scoped the new content as §14, explicitly labeled "Rover_closed_loop/ only" throughout, so a reader doesn't assume `pipeline.c`/`kinematics.c` and the RPi link coexist in one firmware — they don't, the two are sibling projects.
- **Content ported:** physical link + RPi5 OS gotchas (console/Bluetooth/`enable_uart=1`), division of responsibility, SWB mode-arbitration (explicitly noted as a *different* SWB meaning than `Rover/`'s own §2.4 table), the full frame-format tables, failsafe, verification performed (real numbers: 20.0 frames/s, 0 bad), and a file reference. Condensed from `architecture.md` §3's fuller build-log style into `Rover_study.md`'s settled-reference style (its own status legend, tables, no chronological "here's what we tried" narrative).
- **Deliberately left untouched:** §2.6 (IMU) and §9.4's BNO055-on-the-throttle-bus warning still describe the `Rover/` project's *plan* to share I2C1 between the IMU and DACs — moot for `Rover_closed_loop/`, where the IMU lives entirely on the RPi's own I2C1, never touching the STM32 bus at all. Not corrected/annotated since it's still accurate for `Rover/` and out of scope of "the RPi↔STM32 interconnection" — flagging here in case a future full `Rover_study.md` overhaul is requested.

---

### Note: RPi5 UART Pins Confirmed — GPIO14/GPIO15 (Pin8/Pin10)
- **What happened:** User confirmed the RPi5 side of the STM32 UART link uses the primary UART, `GPIO14`/Pin8 (TXD) and `GPIO15`/Pin10 (RXD) — the standard pins, as expected, matching `board.I2C()`-style defaults for the RPi ecosystem.
- **Flagged, not yet resolved by the user:** these pins default to carrying the Linux serial **console** (login shell over UART) on Raspberry Pi OS. Needs disabling via `raspi-config` (serial hardware ON, login shell OFF) before the link is usable — added to the architecture.md §3.6 setup checklist. Not yet confirmed done on the actual RPi5.
- **Applied:** architecture.md pin table (§1) and §3.2 updated with the RPi-side pin mapping and cross-connect table.

---

### Note: RPi5 IMU (DFRobot Fermion BNO055) — Driver Written, Not Yet Hardware-Verified
- **What happened:** Started the RPi-side software (new `RPi_companion/` directory) with an IMU driver for the DFRobot Fermion BNO055, wired by the user to the RPi5's I2C1 bus (VCC→3V3/Pin1, SDA→GPIO2/Pin3, SCL→GPIO3/Pin5, GND→Pin6).
- **Important constraint:** this dev environment is an x86_64 laptop, not the RPi5 (confirmed via `/proc/cpuinfo`) — none of this code has been run on real hardware. It's written to the same library family the old BBB stack used (`adafruit-blinka` + `adafruit_bno055`, per `Old_files/dev_bak/test_imu.py`), since Blinka now supports the Pi 5, but that continuity is a design choice, not a verified fact about this specific board+library+Pi5 combination.
- **Design decisions made without user input, worth double-checking:**
  - I2C address assumed `0x28` (Fermion default, ADR floating/low) — not confirmed via `i2cdetect` on the actual hardware yet.
  - Fault-handling on transient I2C glitches (hold last valid sample, 5-consecutive-failure threshold) deliberately copied from `ads1115.c`'s pattern on the STM32 side, for consistency — reasonable default, not something the user asked for explicitly.
  - Old BBB calibration offsets (`bno055_calibration.json`) explicitly NOT reused — different physical sensor, different mounting. `calibrate_imu.py` produces fresh ones.
- **Next step (user, on the actual RPi5):** run the setup checklist in architecture.md §3.6 — enable I2C, `i2cdetect`, install requirements, `test_imu.py`, then `calibrate_imu.py`. Report back what `i2cdetect` shows and whether readings look sane (heading changes when rotated, etc.) before this driver gets used by anything downstream.

---

### Note: Missing Includes Wiped by CubeMX Regeneration (Found While Implementing Phase 5)
- **What happened:** Before writing the RPi UART firmware, checked whether `main.c` still compiles after the earlier `Rover.ioc`/CubeMX "Generate Code" step (adding USART2). Found `main.c` had **no `#include`** for `ibus.h`, `ackermann.h`, `ackermann_config.h`, `motor.h`, `encoder.h`, `ads1115.h`, `actuator.h`, or `steer_pid.h` — yet the file uses `IBUSData_t`, `Ackermann_Run()`, `Motor_Init()`, etc. throughout, which cannot compile without them.
- **How it was found:** `Debug/Rover.elf` (built before the CubeMX regeneration, per file timestamps) links `Motor_Init`, `IBUS_Init`, `Ackermann_Run`, `ADS1115_Init` symbols successfully — proving these declarations *were* visible to `main.c` at that last successful build. The only place they could have lived is directly above the `USER CODE BEGIN Includes` / `USER CODE END Includes` markers in `main.c` (not inside them) — CubeMX regenerates everything outside USER CODE markers from scratch on every "Generate Code," so a regeneration would silently delete includes sitting outside the block without any error, matching exactly what was found.
- **Root cause:** an earlier edit (prior to this session) added the necessary includes above the USER CODE marker instead of inside it. Today's CubeMX regeneration for the USART2/RPi-link peripheral silently wiped them, per the exact risk `Rover_study.md` §10.3 already warns about ("Regeneration has silently broken this project before").
- **How it was corrected:** re-added all eight includes (plus `rpi_link.h`, `<math.h>`, `<stdio.h>` for the new module) **inside** `main.c`'s `USER CODE BEGIN Includes` / `END Includes` block, so they survive future regenerations.
- **Not independently rebuilt yet** (no `arm-none-eabi-gcc` toolchain available in this environment) — needs a clean build in STM32CubeIDE to confirm.
- **General rule going forward:** after any CubeMX regeneration, check not just the items already in the §10.3 checklist but also that every hand-written module header `main.c` depends on is still included — and that any future manual include additions to `main.c`/`main.h` go *inside* a USER CODE block, never outside one.

---

### Note: SWB Wiring + UART Link Failsafe Implemented (Phase 5)
- **What happened:** Implemented the design recorded in the two notes above (SWB repurposing, UART link-loss→brake) in firmware: `ibus.c`'s `_SwbToAuto()` decodes SWB into `IBUSData_t.auto_mode` (raw < `SWB_THRESH` → MANUAL, else → AUTO — switch-down-is-off is the safer default if SWB is never touched); `rev_speed_pct` is now hardcoded to `1.0f` rather than SWB-derived. `main.c` branches on `rc.auto_mode` (only once `Motor_IsArmed() && rc.rc_ok`, same gate as before): MANUAL calls the existing `Ackermann_Run()` unchanged, AUTO calls the new `Ackermann_RunAuto()` if `RPiCmd_t.link_ok`, else forces `ACK_STATE_BRAKE` directly — implementing the `AUTO_UART_TIMEOUT_US` (300 ms) failsafe designed earlier.
- **New function `Ackermann_RunAuto()`** (`ackermann.c`) takes the RPi's absolute physical targets (steering degrees, speed m/s) instead of joystick `Xn`/`Yn` — deliberately bypasses `Ackermann_Run()`'s stick deadband/curve shaping, since an RPi target isn't a human hand and shouldn't be filtered like one. Clamps the steering target to ±`MAX_STEER_ANGLE_DEG` (Xn is already stick-bounded to ±1.0, but a UART float is not). Below `AUTO_SPEED_DEADBAND_MS` (0.03 m/s), brakes instead of creeping — mirrors `ACK_STICK_DEADBAND`'s role for the stick. Everything downstream (electronic differential off the *measured* angle, throttle mapping, state classification) is identical to the manual normal-driving path.
- **Feedback frame** is sent every main-loop tick (20 Hz) regardless of MANUAL/AUTO, so the RPi's guidance loop can always see measured steering angle + wheel RPM and be state-ready the instant SWB flips to AUTO.
- **Not yet tested on hardware.** No RPi has been connected to PA2/PA3; no build/flash cycle has run since these changes. See architecture.md §3.6 for the full file list touched.

---

### Note: Build Errors — Constants Dropped During Phase 2 Port
- **What happened:** First build attempt of `Rover_closed_loop` after entering the calibration values failed: `ACK_STRAIGHT_DEG` undeclared in `ackermann.c`. Fixed, rebuilt, hit `STEER_INTEGRAL_MAX_PCT` undeclared in `steer_pid.c`. Fixed, rebuilt, hit `KP_STEER`/`KI_STEER`/`KD_STEER` undeclared in `main.c` (at the `SteerPID_Init()` call).
- **Root cause:** all four constants existed in the original `STM_ackermann_backup/Inc/ackermann_config.h` but were silently dropped when that file was ported into `Rover_closed_loop/Core/Inc/ackermann_config.h` during Phase 2 — not caused by the calibration/deadband edits made in this session.
- **How it was found:** iteratively, one compile error at a time (STM32CubeIDE `make -j12`), cross-checked each time against every identifier used across `ackermann.c`, `actuator.c`, `ads1115.c`, `steer_pid.c`, `main.c` and their headers vs. what's defined in `config.h`/`ackermann_config.h`.
- **How it was corrected:** re-added all four with their original backup values — `ACK_STRAIGHT_DEG=1.0f`, `STEER_INTEGRAL_MAX_PCT=50.0f`, `KP_STEER=5.0f`, `KI_STEER=0.1f`, `KD_STEER=0.5f` — to `ackermann_config.h`. Note: `KP/KI/KD_STEER` and `steer_pid.c` as a whole are effectively dead code right now since `main.c`'s actual actuator control (step 6 of the loop) is bang-bang, not PID-driven — `SteerPID_Init()` is called at boot but `SteerPID_Update()` is never called. Confirmed building 0 errors, 0 warnings after these fixes.
