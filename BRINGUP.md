# Hardware Bringup — Welding Trainer Pantograph

**This document gates active force.** Closed-loop motor force must NOT be
enabled on the rig until every step below has passed and been initialed. The
firmware enforces part of this (closed-loop will not engage until
`polarity_ok` is set), but the steps that protect *people and the
mechanism* are procedural and live here.

Read the whole document before powering the motors. Keep one hand on the
spacebar (CPU E-stop) and know where the motor supply switch is for every
powered step.

> Status legend per step: **PURPOSE** / **PROCEDURE** / **PASS CRITERIA** /
> **ABORT IF** / initials + date.

---

## 0. Pre-flight (no motor power)

Do these with the motor power supply **off** (logic/USB only).

### 0.1 — CAD dimensions match the physical rig
- **PURPOSE:** `kinematics.h` now carries the **measured** dimensions:
  `kBaseMm=87.0` (8.7 cm) and
  `kL1Mm=kL2Mm=kL3Mm=kL4Mm=152.4` (6 in). Forward
  kinematics and the Jacobian are correct in form; wrong dimensions silently
  distort the whole workspace.
- **PROCEDURE:** Confirm the baked constants in
  `arduino/welding_common/kinematics.h` match the physical rig with calipers (all
  four moving links pivot-to-pivot, and the motor-to-motor base spacing). If the
  rig differs, update the constants, then recompile + reflash **both** boards
  (calibration is firmware-baked — recalibration = reflash).
- **PASS:** Baked dimensions match the physical rig to ±0.5 mm.
- **ABORT IF:** A measured link disagrees with the baked value beyond tolerance
  and cannot be reconciled — a wrong link length silently mirrors or distorts the
  whole workspace.
- [ ] initials ______ date ______

### 0.2 — Pin map matches the wiring (MR analog sensor)
- **PURPOSE:** A wrong pin number is the simplest failure: nothing moves, the
  sensor reads garbage. The firmware now reads the **MR analog sensor on
  `A2`** (`analogRead` + software flip-unwrap + per-kit linear fit) — the
  quadrature `Encoder.h` path is GONE, and the old D2/D4 encoder pins are FREE.
- **PROCEDURE:** The pin map is the **Hapkit native** one (verified against the
  Hapkit shield pinout reference) — confirm the harness matches `config.h`:
  rotating motor on the **Motor 1 channel (PWM D5 / DIR D8)** (both boards); the
  **ERM on the Motor 2 channel (PWM D6 / DIR D7)** (MAIN only); MR sensor on
  **A2** (both boards); the MAIN↔secondary link is now **hardware I2C/TWI** —
  MAIN **A4↔A4** secondary (SDA), **A5↔A5** (SCL), shared **GND**, with ONE
  pull-up pair to +5V (one on SDA, one on SCL) — **3.3 kΩ** works well
  (a bit stronger/faster than the typical 4.7 kΩ, and 400 kHz-ready). Same-to-same, **NOT
  crossed**; keep SDA/SCL short and routed away from motor leads. **D2/D3 are now
  FREE.** The shield routes D4 (SD-SS), D9/D10/A0/A1 (Grove), D11–D13 (SD-SPI),
  A3 (FSR) — leave them unused.
- **PASS:** Every signal lands on its `config.h` pin; motor on Motor-1 (D5/D8),
  ERM on Motor-2 (D6/D7), MR on A2, link on **A4/A5 (SDA/SCL) ↔ A4/A5 + shared GND
  + the 4.7 kΩ pull-up pair**.
- **ABORT IF:** Any signal lands on a shield-routed pin (D4/D9/D10/D11–D13/A0/A1/A3)
  or the wrong channel.
- [ ] initials ______ date ______

### 0.2b — MR sensor boot tare (hold centerline at power-on)
- **PURPOSE:** The MR sensor is relative; each board resolves its zero by a
  **boot tare** — at power-on it maps the current pose to the centerline
  reference (`kThetaTareRefRad = π/2`). Tare at the wrong pose offsets every
  reported angle and rotates the Jacobian.
- **PROCEDURE:** Before powering EITHER board, position the end-effector at the
  **centerline pose** (both proximal links vertical, θ₁=θ₅≈π/2) and hold it
  steady. Power on; the firmware tares in `setup()` before the control loop
  starts. If you power on at the wrong pose, power-cycle at the correct one.
- **PASS:** Right after boot, the CPU HUD shows the end-effector at ~centerline
  (reported tip near the top of the workspace, ry≈0).
- **ABORT IF:** Reported position is wildly off at a known pose → re-tare
  (power-cycle at centerline); confirm in §2.1 before enabling force.
- [ ] initials ______ date ______

### 0.5 — ERM on the Motor-2 H-bridge channel (drive stage built-in)
- **PURPOSE:** An ERM draws ~50–100 mA and is inductive; an AVR pin sources
  ~40 mA with no flyback path (integration-concerns §5). Driving it pin-direct
  browns out or damages the pin.
- **RESOLVED BY DESIGN:** the ERM rides the Hapkit **Motor 2 H-bridge
  channel (PWM D6 / DIR D7)** — the H-bridge IS the drive + flyback stage, so no
  separate 2N2222 is needed. Firmware holds DIR LOW and modulates PWM; the cue is
  clamped to the tested-safe band **28–70** in the of-app (`computeErmBreakdown`).
- **PROCEDURE:** Confirm the ERM is wired to the Motor-2 channel (D6/D7), NOT to a
  bare logic pin. Confirm the of-app commands stay in 28–70 (they do).
- **PASS:** ERM is on the Motor-2 H-bridge channel; commanded PWM stays ≤70.
- **ABORT IF:** The ERM is wired directly to a bare logic pin (no H-bridge), or
  its commanded PWM can exceed ~70.
- [ ] initials ______ date ______

### 0.3 — Confirm the COM port (serial)
- **PURPOSE:** The CPU app currently opens serial **device 0 blindly**
  (`ofSerial::setup(0, 115200)`). With more than one USB-serial adapter
  present it may grab the wrong one.
- **PROCEDURE:** In Device Manager, note the Arduino's COM port. If it is not
  the first enumerated serial device, either unplug other adapters for the
  session or extend the CLI to `--source=serial:COMn` (one parse change).
- **PASS:** `--source=serial` connects to the Arduino (banner goes GREEN).
- **ABORT IF:** The banner stays RED or connects to the wrong device.
- [ ] initials ______ date ______

### 0.4 — Watchdog + latched-safe sanity (logic only)
- **PURPOSE:** Confirm the safety state machine before any motor can move.
- **PROCEDURE:** With motor power still OFF, run the CPU app against the rig.
  Confirm the Arduino boots into `SAFE_LATCHED` with `polarity_ok == false`
  (RAM-only, cleared every power-up). Confirm the CPU HUD shows the
  safety banner.
- **PASS:** Fresh boot = latched, force-disabled, ERM off.
- **ABORT IF:** The board comes up in any state that would allow force on a
  power cycle.
- [ ] initials ______ date ______

---

## 1. JOG_MODE polarity verification (motor power ON, low PWM)

**This is the single most important safety step.** An inverted motor
sign turns the force loop into positive feedback → runaway. Closed-loop force
stays disabled until this passes.

**Two boards, two runs:** each motor lives on its own Hapkit, so JOG_MODE
is built and run **per board**. §1.1 is the **main** (right) motor — set
`kMotorRightSign` / `kEncRightSign`. §1.2 is the **secondary** (left) motor — flash
the secondary with `WELDING_JOG_MODE`, set `kMotorLeftSign` / `kEncLeftSign`.
Closed-loop force engages only when **both** boards are confirmed and the link is
fresh (the main gates on its own `polarity_ok` AND the secondary's ready bit AND
link freshness).

### 1.1 — Main (right) motor polarity  [main board, `kMotorRightSign`/`kEncRightSign`]
- **PURPOSE:** Confirm a positive PWM command moves the right (main-local) angle
  sensor in the positive direction.
- **PROCEDURE:** Build/flash the **main** firmware with `WELDING_JOG_MODE` enabled.
  It jogs only the local right motor at low PWM (well under the mechanically-safe
  clamp). Observe the right angle reading.
- **PASS:** Positive PWM → reading increases. Negative PWM → decreases.
- **ABORT IF:** Sign is inverted → swap the motor leads OR flip `kMotorRightSign`
  (and/or `kEncRightSign`) in `arduino/welding_common/config.h` and reflash.
  Re-test before proceeding. Do NOT "remember to account for it later."
- [ ] initials ______ date ______

### 1.2 — Secondary (left) motor polarity  [secondary board, `kMotorLeftSign`/`kEncLeftSign`]
- Same procedure for the **secondary**: flash `welding_secondary` with
  `WELDING_JOG_MODE`, jog the left motor, set `kMotorLeftSign`/`kEncLeftSign` in
  `config.h`. (Signs live in `config.h`, not a `motor_config.h` — that file does
  not exist.)
- [ ] initials ______ date ______

### 1.3 — Set polarity_ok
- **PURPOSE:** Tell the firmware polarity is confirmed so closed-loop may later
  engage.
- **PROCEDURE:** Set `polarity_ok` by sending `MSG_ARM` — **press Enter** on a
  fresh boot (`ofApp.cpp:711` arms on Enter unconditionally, and the app boots
  un-latched at `ofApp.h:130`; the prior "Space-then-Enter" dance is no longer
  required). The spacebar still latches a
  software E-stop, and Enter re-arms from it. Remember: `polarity_ok` is RAM-only
  and clears on the next power cycle — every power-up re-requires §1.
- **PASS:** Firmware reports polarity confirmed (status bit / HUD).
- [ ] initials ______ date ______

### 1.4 — Torque→PWM map: capstan ratio + sqrt law  *(law DONE; calibration DEFERRABLE)*
- **PURPOSE:** The sqrt+capstan law is **already implemented**
  (`inner::torqueToSignedPwmCapstan`, `inner_loop.h:142`) — it supersedes the old
  linear `kTorqueToPwmScale`. `Tp=(rp/rs)·|tau|`, `duty=sqrt(|Tp|/k)`,
  `pwm=sign·duty·255`. What's left is **per-kit calibration of the two constants**,
  which only scales force *magnitude* — it cannot change force *sign/stability*.
- **DEFERRABLE FOR FIRST FORCE:** The baked values are the stock Hapkit template
  defaults (`kCapstanRpOverRs=0.005/0.077`, `kMotorTorqueConstNm=0.0183`,
  `config.h:161-162`), NOT measured on this rig. For a first feel-test that is
  fine — start at a reduced force ceiling (§4.2) and tune by feel. Measure the
  real values only when you want trustworthy *Newtons*.
- **PROCEDURE (when calibrating):** Measure the capstan ratio `rs/rp`
  (sector R / shaft r ≈ 7.8–15) and the per-kit torque constant (the `0.0183`
  N·m-at-duty²=1 equivalent); update `config.h:161-162`; record them here.
- **PASS:** Commanded duty tracks √torque; small forces are renderable, not lost
  in a low-torque deadband.
- **NOTE:** A wrong constant scales every rendered force but never reverses it —
  no runaway risk from this step (that is §1.1/§1.2's job).
- [ ] initials ______ date ______  *(may be marked DEFERRED for first force)*

### 1.5 — Inter-board link bringup (now hardware I2C)
- **PURPOSE:** Confirm the primary↔secondary **I2C** link and its fail-safe before any
  closed-loop force. A stale θ_left corrupts the Jacobian, so link loss must cut
  force, not coast on the last value.
- **PRECHECK (bus scan, no real firmware needed):** flash `arduino/link_diag` (the
  I2C bus scanner) to the MAIN with the secondary powered (it claims `0x08`), and
  confirm the main prints **`found 0x08  <- secondary`** over USB Serial. Nothing
  found → check the 3.3/4.7 kΩ pull-ups, that SDA/SCL aren't swapped, and the shared
  GND. ⚠ The scan proves only *addressing* — the
  LIVE link ALSO needs nested interrupts in the 1 kHz ISR: without it the software-float
  `fwdKin` ISR starves the interrupt-driven TWI controller and θ_left freezes even though the scan
  passes. Use `arduino/link_probe` (does the real `requestFrom`+unpack+write, prints got/CRC-ok/
  wErr) to tell a wiring fault from this ISR/TWI starvation.
- **PROCEDURE:** Wire **A4↔A4 (SDA), A5↔A5 (SCL), shared GND** + the 4.7 kΩ pull-up
  pair (§0.2; same-to-same, NOT crossed). Power both. With the secondary in its
  normal (non-JOG) build, move the **left** joint by hand and confirm the main's
  reported (x,y) responds (θ_left is arriving). Confirm each board's boot self-test
  ping twitches its motor once. Then, with force enabled at a reduced clamp, **pull
  the SDA wire mid-run** (or power the secondary off) and confirm the left motor goes
  dead within `kLinkWatchdogMs` (50 ms) and the main force-disables — AND that the
  main does **NOT hang** (`Wire.setWireTimeout` fires) and **recovers** when you
  reconnect.
- **PASS:** θ_left tracks the hand; pulling SDA / powering off the secondary cuts
  both motors within 50 ms; the main never hangs and recovers on reconnect; nothing
  runs away.
- **NOTE (motors-off rehearsal):** the link-loss behavior can be rehearsed safely with
  motors off first — pull **A4 (SDA)** mid-run and confirm θ_left freezes while θ_right keeps
  tracking, the main does **NOT** hang (`setWireTimeout` fires), and it recovers on reconnect.
  That rehearsal does NOT cover the force-cut criterion: the box stays unchecked until
  motors-die-within-50-ms is confirmed with force enabled.
- **ABORT IF:** The left motor holds force after link loss, OR the main hangs when
  SDA is pulled (setWireTimeout not effective), OR θ_left does not
  track → fix before enabling full force.
- [ ] initials ______ date ______

---

## 2. Encoder direction + forward-kinematics reality check

### 2.1 — Hand-move tracking
- **PURPOSE:** Catch the pantograph upper/lower-root mirror bug and any
  encoder direction error before force is enabled.
- **PROCEDURE:** With motors un-powered (back-drivable) or in a safe low-power
  hold, move the end-effector by hand to several known workspace points
  (corners + center). Compare the firmware's reported (x,y) against the
  physical position.
- **PASS:** Reported (x,y) tracks reality across the workspace, including
  points across the symmetry line (no mirror flip).
- **ABORT IF:** Position appears mirrored across the centerline → the
  fwd-kinematics root choice is wrong; fix in `kinematics.h`.
- [ ] initials ______ date ______

### 2.2 — q15 range verification
- **PURPOSE:** The protocol carries position as q15 ↔ ±256 mm and velocity as
  q15 ↔ ±512 mm/s. The reported position is the **end-effector tip
  re-centered** on the workspace origin (`kReportOriginYMm`) — chosen so the
  workspace fits ±256 mm (the absolute tip is ~542 mm from the motor midpoint).
  Confirm the real workspace fits and that the CPU applies the **same** origin.
- **PROCEDURE:** At known positions, confirm the decoded mm on the CPU matches
  the physical mm. Move at a known hand speed; sanity-check decoded mm/s.
- **PASS:** Decoded units match physical reality within LSB resolution.
- **ABORT IF:** Workspace exceeds ±256 mm or velocities exceed ±512 mm/s →
  re-scale the q15 mapping.
- [ ] initials ______ date ______

---

## 3. Watchdog trip + re-arm (motor power ON)

### 3.1 — Heartbeat-loss trips SAFE_LATCHED
- **PURPOSE:** Confirm the dead-man watchdog.
- **PROCEDURE:** With the system running, cut the CPU heartbeat (close the app
  or pause the heartbeat thread). Time the response.
- **PASS:** Within 100 ms the motors go to zero force and ERM goes off; the
  firmware enters `SAFE_LATCHED`; the CPU HUD shows "SAFETY TRIPPED."
- **ABORT IF:** Motors hold force longer than 100 ms after heartbeat loss.
- [ ] initials ______ date ______

### 3.2 — Re-arm requires ARM + Enter
- **PURPOSE:** Confirm recovery is deliberate, not automatic.
- **PROCEDURE:** From `SAFE_LATCHED`, attempt to resume. Confirm it requires
  BOTH an `ARM` message from the CPU AND the user pressing Enter.
- **PASS:** No auto-resume; force re-enables only after the explicit two-part
  re-arm.
- **ABORT IF:** Force re-engages on reconnect alone (surprise-force hazard).
- [ ] initials ______ date ______

### 3.3 — Spacebar E-stop
- **PURPOSE:** Confirm the CPU-side E-stop.
- **PROCEDURE:** During a powered hold, press spacebar.
- **PASS:** Immediate drop to zero force + ERM off + SAFE_LATCHED; re-arm needs
  Enter.
- [ ] initials ______ date ______

---

## 4. Force ramp + first low-power active force

Only reach this section once §0–§3 are fully initialed.

### 4.1 — Force ramp at session start
- **PURPOSE:** No instant max-force surprise (safety baseline).
- **PROCEDURE:** Start a session. Observe the commanded force at t=0.
- **PASS:** Force ramps from zero over the ramp window; no step to full force.
- **ABORT IF:** Force jumps to a large value at session start.
- [ ] initials ______ date ______

### 4.2 — First closed-loop run at reduced clamp
- **PURPOSE:** First real active-force test, de-risked.
- **PREFERRED KNOB:** Temporarily lower **`kForceCmdMaxN`** (`config.h`,
  the cable-slip ceiling, default 5 N) to **~1.5 N** and reflash the primary. This
  caps force in the units you *feel*, and `inner::clampForceMagN` scales the
  command vector while preserving *direction* — so the restoring-vs-repelling
  read stays valid. (Lowering `kPwmMaxClamp` also works but limits duty, not
  force, and is coarser.) **Safety floor:** even at the full 5 N ceiling a
  sign-error runaway is bounded to ~5 N of command (and the capstan cable slips
  ~4 N), so the worst case is a push you can resist by hand, not a full-torque
  slam — but reduce it anyway for the first run.
- **PROCEDURE:** Hand on spacebar. Load `linear.json`. Trace the path slowly and
  feel for: (a) force pulls *toward* the line (restoring, not repelling — a
  repelling force means a residual sign error from §1), (b) ERM detent
  strengthens near target speed, (c) blow-through ERM spike when you leave the path.
- **PASS:** Forces restore toward the path; ERM cues behave; nothing runs away.
- **ABORT IF:** Any force pushes *away* from the path, or oscillates with
  growing amplitude → spacebar immediately, return to §1 (polarity) and the
  gain sliders.
- [ ] initials ______ date ______

### 4.3 — Restore full ceiling
- Once 4.2 is clean, restore `kForceCmdMaxN` to 5 N (and `kPwmMaxClamp` to 200 if
  you touched it), reflash, and re-confirm 4.1 + 4.2 at full force, hand on spacebar.
- [ ] initials ______ date ______

---

## After bringup

- Record the measured CAD dimensions and any sign flips in a commit so the
  firmware-baked constants are the source of truth.
- Run the latency characterization on the live rig:
  encoder→ERM ≤ 5 ms, encoder→screen ≤ 30 ms, encoder→audio ≤ 20 ms.
- Consider the EEPROM-persisted `polarity_ok` follow-up only after the
  bringup procedure has been stable across several sessions.

---

*Active force is gated on this checklist.*
