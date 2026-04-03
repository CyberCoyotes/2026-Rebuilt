# Flywheel Tuning Guide & Protocol

**For:** CyberCoyotes 4829 — 2026-Rebuilt
**Hardware:** 2x Kraken X60 (TalonFX)
**Control Mode:** VelocityVoltage (no CANivore required)

Work through the sections in order. Each phase depends on the previous one being correct.

---

## Dashboard Values You Will Use

| Elastic Widget | NT Key | What It Tells You |
|---|---|---|
| Flywheel RPM | `Shooter/FlywheelRPM` | Actual motor RPM right now |
| Target RPM | `Shooter/TargetFlywheelRPM` | What the code is asking for |
| Flywheel Error | `Shooter/FlywheelError` | target − actual (goal: near 0) |
| Applied Volts | `Shooter/FlywheelAppliedVolts` | Voltage sent to motor (max ~12 V) |
| Is Ready | `Shooter/IsReady` | Green = ready to feed a ball |
| State | `Shooter/State` | IDLE / READY / PASS / EJECT |

---

## Before You Start

**Code files you will edit:**
- Gains → `src/main/java/frc/robot/subsystems/shooter/ShooterIOHardware.java` (lines 94–109, `Slot0`)
- RPM presets → `src/main/java/frc/robot/Constants.java` (lines 105–163, `Shooter` class)
- Tuning command → `src/main/java/frc/robot/RobotContainer.java` (line 119)

**How to spin the flywheel for tuning:**
Hold **Right Bumper** → flywheel spins at the RPM set in `tuneFlywheelCommand(3300)`.
Release **Right Bumper** → flywheel stops.
Change the number in `tuneFlywheelCommand()` to test different RPMs and redeploy.

**Safety:** Robot must be on the ground with bumpers on. Never tune with a ball in the chamber until Phase 4.

---

## Phase 1 — kV (Feedforward) Tuning

**What kV does:** Provides the baseline voltage needed to spin at a given speed. Get this right first so kP has less work to do.

**Goal:** With kP = 0, `FlywheelRPM` reaches close to `TargetFlywheelRPM` at steady state.

### Steps

1. In `ShooterIOHardware.java` Slot0, set:
   ```java
   config.Slot0.kP = 0.0;   // Zero out for now
   config.Slot0.kV = 0.12;  // Starting point
   ```
2. Deploy. Enable. Hold Right Bumper (spins at 3300 RPM).
3. Wait 3–4 seconds for speed to settle.
4. Read `FlywheelRPM` and `FlywheelError` on dashboard.

| What You See | What To Do |
|---|---|
| `FlywheelRPM` < 3300, `FlywheelError` positive | Increase `kV` by 0.005 |
| `FlywheelRPM` > 3300, `FlywheelError` negative | Decrease `kV` by 0.005 |
| `FlywheelAppliedVolts` already at 11–12 V | System is saturated; lower target RPM first |

5. Redeploy after each change. Repeat until `FlywheelError` stays within **±200 RPM** at steady state.
6. Record your kV: **kV = ______**

**Typical Kraken X60 range in voltage mode: kV ≈ 0.110 – 0.130 V/RPS**

---

## Phase 2 — kP (Proportional) Tuning

**What kP does:** Corrects the remaining error that kV alone cannot fix. Too low = slow/inaccurate. Too high = oscillation (RPM bouncing).

**Goal:** `FlywheelError` drops to < ±100 RPM with no oscillation.

### Steps

1. With the kV value locked from Phase 1, set:
   ```java
   config.Slot0.kP = 0.05;  // Start low
   ```
2. Deploy. Enable. Hold Right Bumper.
3. Watch `FlywheelError` after settling (~3 seconds):
   - Still >100 RPM error → increase `kP` by `0.02`
   - RPM hunting (goes up, overshoots, comes back, repeats) → **oscillation — back off kP by 20%**
4. Repeat until `FlywheelError` stays < **±100 RPM** without hunting.

**Oscillation check:** Watch `FlywheelRPM` for at least 5 seconds. If it oscillates rhythmically (sawtooth pattern), kP is too high.

5. Record your kP: **kP = ______**

**Typical range: kP ≈ 0.05 – 0.20 V/RPS**

---

## Phase 3 — Ready Check Validation

**What this checks:** The `IsReady` flag controls when the indexer can fire. Verify it triggers at the right time.

Current tolerance: **±10%** of target RPM
`Constants.java` line: `FLYWHEEL_TOLERANCE_PERCENT = 0.10`

### Steps

1. Deploy with Phase 1–2 gains.
2. Hold Right Bumper (3300 RPM target).
3. Watch `Shooter/IsReady` on dashboard — it should turn **true within 1–2 seconds**.

| Problem | Fix |
|---|---|
| `IsReady` never goes true even at target RPM | Check `FlywheelError` is actually < 330 (10% of 3300); if yes, check NT key name |
| `IsReady` goes true immediately before flywheel spins up | Tolerance is too loose; change to `0.05` (5%) in Constants.java |
| `IsReady` takes > 3 seconds | Gains are too weak; increase kV or kP slightly |

**Acceptable:** `IsReady = true` within 1.5 seconds of spinning up to steady state.

---

## Phase 4 — Preset RPM Validation

**What this does:** Verifies each shot preset RPM actually scores consistently.

### Procedure (repeat for each preset below)

1. Change `tuneFlywheelCommand(TARGET_RPM)` in `RobotContainer.java` line 119 to the preset RPM.
2. Deploy. Hold Right Bumper. Wait for steady state.
3. Confirm `FlywheelRPM` ≈ `TargetFlywheelRPM` and `FlywheelError` < 150.
4. Hold trigger to enter READY state → manually feed a ball → observe scoring.
5. Adjust RPM in `Constants.java` and repeat until 3 of 3 shots score.

### RPM Presets to Validate

| Preset | RPM Constant | Current Value | Tuned Value | Notes |
|---|---|---|---|---|
| CLOSE | `CLOSE_RPM` | 2750 | | Best starting point |
| TOWER | `TOWER_RPM` | 3200 | | TODO in code |
| TRENCH | `TRENCH_RPM` | 3200 | | TODO in code |
| FAR | `FAR_RPM` | 3800 | | Was 4000, reduced |
| PASS | `PASS_RPM` | 4000 | | |

**Tip:** Tune CLOSE first (shortest distance, most forgiving). Use it to verify the complete shot sequence before moving to longer distances.

**Shot adjustment guide:**
- Ball lands **short / hits low**: increase RPM by 100–200
- Ball lands **long / hits high**: decrease RPM by 100–200
- Ball lands consistently off-center: check hood angle (see `TUNING.md`)

---

## Phase 5 — Vision Interpolation Table

**What this does:** Sets the RPM and hood angle the robot uses at each measured distance when using `visionAlignAndShoot()`.

Located in `ShooterSubsystem.java` lines 493–514.

### Procedure

1. Use `Vision/Distance_m` on Elastic to confirm measured distance (see `TUNING.md §3` for distance calibration).
2. Park robot at exactly **1.0 m, 2.0 m, 3.0 m, 4.0 m, 5.0 m, 6.0 m** from hub tag.
3. At each distance, use Phase 4 procedure to find the RPM that scores consistently.
4. Update `FLYWHEEL_RPM_MAP.put(distance, tunedRPM)` in `ShooterSubsystem.java`.

### Tuning Log

| Distance (m) | Tuned RPM | Hood (rot) | Confirmed Scoring | Notes |
|---|---|---|---|---|
| 1.0 | | | ☐ | |
| 2.0 | | | ☐ | |
| 3.0 | | | ☐ | |
| 4.0 | | | ☐ | |
| 5.0 | | | ☐ | |
| 6.0 | | | ☐ | |

**Tips:**
- Tune 1.0 m and 6.0 m endpoints first — interpolation fills in the middle.
- If all distances are biased (all long or all short), recheck camera angle in `Constants.Vision` before adjusting every RPM.
- Mark each confirmed row with `// Tuned` comment in code.

---

## Final Validation Checklist

Before locking in gains for competition:

- [ ] `FlywheelError` < ±100 RPM at steady state for all preset RPMs
- [ ] `FlywheelAppliedVolts` stays below 11 V at all tested RPMs (not saturating)
- [ ] `Shooter/IsReady` goes true within 1.5 seconds of reaching target
- [ ] No oscillation (RPM hunting) visible in `FlywheelRPM` over 5+ seconds
- [ ] Each preset scores 3 of 3 practice shots before locking in
- [ ] Post-shot RPM dip < 10% and recovery < 300 ms (watch during rapid-fire)
- [ ] `EJECT_RPM` (-1500) clears jams without back-feeding
- [ ] kP and kV values recorded in this file:

```
// Final tuned gains — update when changed
Slot0.kV = ______
Slot0.kP = ______
Date tuned: __________
```

---

## Quick Reference: Gain Effects

| Symptom | Likely Cause | Fix |
|---|---|---|
| High steady-state error, kP = 0 | kV too low | Increase kV |
| RPM oscillates (sawtooth) | kP too high | Reduce kP by 20% |
| Good steady-state but slow rise | kV or kP too low | Increase kV first, then kP |
| `AppliedVolts` maxed (11–12 V) | Target RPM too high OR current limit too low | Lower RPM target or raise stator limit |
| RPM drops a lot after each shot | Needs higher kP or higher stator peak limit | Increase kP by 0.02; or raise `StatorCurrentLimit` |
| Shots short even at correct RPM | Mechanical slip, hood angle, or ball grip issue | Check hood and wheel grip before changing RPM |

---

*See also: `TUNING.md` for full vision + hood tuning protocol.*
