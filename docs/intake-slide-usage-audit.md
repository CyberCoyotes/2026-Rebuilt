# IntakeSubsystem slide controls usage audit (2026-04-06)

## Scope

This audit answers: **which slide methods, slide commands, and slide command factories are truly in use by the robot's active control flows**.

"Active control flows" here means calls that originate from:

- `RobotContainer` driver/operator bindings
- `AutoRoutines` choreo event hooks
- `FuelCommands` (the command class imported by `RobotContainer`)

## Actively used slide command factories (directly bound/called)

### Driver/Operator bindings (`RobotContainer`)

- `retractSlidesSlowHeldCmd()`
- `retractSlidesIncrementalCmd()`
- `manualSlideExtendHoldCmd()`
- `manualSlideRetractHoldCmd()`

### Autonomous/event paths (`AutoRoutines` + `FuelCommands`)

- `intakeFuelTimer(...)`
- `retractSlidesAuton()`

### Utility method used by `FuelCommands` pump logic

- `setSlidesToPosition(...)`

## Methods currently reachable through active flows

These lower-level methods are not directly bound, but are **reachable** through the active command factories above:

- `extendSlidesFast()` (via `intakeFuel()` / `intakeFuelTimer()`)
- `retractSlidesSlow()` (via `retractSlidesSlowHeldCmd()` / `retractSlidesAuton()`)
- `retractSlidesIncremental()` (via `retractSlidesIncrementalCmd()` / `retractSlidesAuton()`)
- `nudgeSlides(...)` (via `manualSlideExtendHoldCmd()` and `manualSlideRetractHoldCmd()`)

## Slide APIs with no active call path from the current control graph

The following slide-related APIs exist in `IntakeSubsystem`, but currently have no call path from `RobotContainer`, `AutoRoutines`, or `FuelCommands`:

- `moveSlidesHome()` / `moveSlidesHomeCmd()`
- `retractSlidesFast()` / `retractSlidesFastCmd()`
- `extendSlidesCmd()` / `retractSlidesCmd()` (backward-compat aliases)
- `retractSlidesSlowCmd()` (non-held single-shot variant)
- `retractSlidesStack()` (legacy workaround sequence)
- `stopSlide()`
- `extendSlides()` / `retractSlides()` (backward-compat aliases)

## Suggested cleanup order (safe, incremental)

1. Keep all **actively used** methods listed above.
2. Remove obvious aliases first (`extendSlidesCmd`, `retractSlidesCmd`, `extendSlides`, `retractSlides`) after verifying no dashboard/button bindings depend on command names.
3. Evaluate whether `retractSlidesStack()` is still needed in practice; remove if reliability issue is no longer present.
4. Decide on one stow path (`retractSlidesFast` vs `moveSlidesHome`) and remove the redundant one.
5. If no future direct motor stop behavior is needed, remove `stopSlide()` from subsystem + IO interface.

## Repro commands used for this audit

```bash
rg -n "intake\.[A-Za-z0-9_]*slide|intake\.[A-Za-z0-9_]*Slides|intake\.setSlidesToPosition|intake\.intakeFuel|intake\.retractSlidesAuton" \
  src/main/java/frc/robot/RobotContainer.java \
  src/main/java/frc/robot/AutoRoutines.java \
  src/main/java/frc/robot/commands/FuelCommands.java

rg -n "\b(extendSlidesFast|retractSlidesFast|moveSlidesHome|retractSlidesSlow|extendSlides|retractSlides|stopSlide|setSlidesToPosition|nudgeSlides|retractSlidesIncremental|extendSlidesFastCmd|retractSlidesFastCmd|moveSlidesHomeCmd|retractSlidesSlowCmd|retractSlidesSlowHeldCmd|extendSlidesCmd|retractSlidesCmd|retractSlidesIncrementalCmd|manualSlideNudgeHoldCmd|manualSlideExtendHoldCmd|manualSlideRetractHoldCmd|retractSlidesStack|retractSlidesAuton)\s*\(" src/main/java
```
