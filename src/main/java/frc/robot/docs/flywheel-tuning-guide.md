## Flywheel Tuning Guide & Testing Protocol

Team-Focused, Match-Ready Procedure for CTRE Kraken / Phoenix 6 Systems

### Objective

Develop a flywheel system that: - Spins up quickly - Recovers rapidly
after each shot - Maintains consistent exit velocity late in match -
Avoids brownouts or current-induced performance decay

This guide assumes: - VelocityVoltage control mode - FOC enabled (Pro
license) - Multi-motor flywheel (e.g., 3 Krakens) - Command-based Java
framework


## SECTION 1 --- Required Signals to Log

From Lead Flywheel Motor:

Required: - RotorVelocity - ClosedLoopError (velocity) - SupplyCurrent -
StatorCurrent - MotorVoltage (or Applied Output) - SupplyVoltage -
DeviceTemp

Recommended: - Shot event boolean (IndexerFeeding or ShotRequested) -
Battery voltage (RoboRIO or PDH)

## SECTION 2 --- Baseline Configuration

Control Mode: VelocityVoltage.withEnableFOC(true)

Initial Current Limits (per motor): - Stator: 90A continuous, 120A peak
(0.3--0.5s) - Supply: 45A continuous, 60A peak (0.3--0.5s)

Initial Gains (starting point only): - kP: modest (avoid oscillation) -
kV: calculated from free speed - kA: small or zero initially

## SECTION 3 --- Spin-Up Test (No Ball)

Procedure: 1. Spin to target RPM. 2. Verify stable hold for 3 seconds.
3. Record: - Steady-state error - Holding current - Voltage used

Expected: - Error \< ±50 RPM - Stable current - No oscillation

If unstable: - Reduce kP If slow to reach target: - Increase stator
limit slightly - Increase acceleration allowance

## SECTION 4 --- Single Shot Impact Test

Procedure: 1. Reach stable target RPM. 2. Fire one ball. 3. Measure: -
RPM dip magnitude - Recovery time - Current spike

Targets: - RPM dip ideally \< 10% - Recovery \< 300 ms

If dip too large: - Increase stator peak - Slightly increase kP - Check
mechanical slip

If recovery slow: - Increase kV slightly - Verify supply current not
limiting

## SECTION 5 --- Rapid Fire Test

Procedure: 1. Fire 3--5 balls spaced closely. 2. Log pre-shot RPM for
each.

Goal: Pre-shot RPM should remain consistent.

If later shots are softer: - Add ready-stable gating (150--250 ms) -
Prevent feeding until RPM within tolerance - Consider slight target RPM
bump during feed window

## SECTION 6 --- End-of-Match Simulation

Procedure: 1. Drive aggressively for 60 seconds. 2. Immediately fire 5
balls. 3. Compare first shot vs last shot.

Check: - Pre-shot RPM consistency - Recovery time drift - Temperature
rise - Supply voltage sag

If degradation observed: - Lower supply peak slightly - Improve gating -
Verify cooling - Check belt tension and wheel grip

## SECTION 7 --- Advanced Shot Gating Logic

Feed ONLY when: - Velocity within tolerance - ClosedLoopError small -
Stable for minimum duration (150--250 ms)

Optional: Lockout feed for 150--250 ms after shot unless RPM recovered.

This prevents panic-feeding late match.

## SECTION 8 --- Data Interpretation Guide

Voltage Sag but No Brownout: Normal. Adjust supply limit if severe.

High Stator Current but Slow Recovery: Possible belt slip or
conservative acceleration.

ClosedLoopError Remains High: Increase kP or stator peak.

Same RPM but Softer Shot: Mechanical slip or feed timing issue.

## SECTION 9 --- Final Validation Checklist

Before competition:

-   10 consecutive shots consistent
-   Recovery time stable
-   No CAN faults
-   No brownout flags
-   Temperature stable under repeated fire
-   Drivers confirm shot consistency

## Summary

Most late-match softness is caused by: - Feeding before recovery -
Conservative tuning - Thermal drift - Mechanical slip

Electrical brownouts are rarely the true cause if drivetrain behaves
normally.

Tune with data. Enforce recovery. Protect your battery. Trust the logs.

------------------------------------------------------------------------

End of Guide.
