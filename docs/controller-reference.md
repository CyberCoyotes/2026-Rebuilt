# Controller Reference

Quick reference for Xbox controller bindings configured in `RobotContainer.java`.

---

## Controller Ports

| Port | Controller | Purpose |
|------|------------|---------|
| 0 | Driver | Drivetrain movement and all currently active mechanism controls |
| 1 | Operator | Reserved / currently unused in bindings |

---

## Driver Controller (Port 0)

Controls drivetrain movement plus shooter, intake, and climber actions.

### Joysticks

| Input | Function | Details |
|-------|----------|---------|
| **Left Stick** | Translate | Forward/backward (Y-axis), Left/right (X-axis) |
| **Right Stick (X)** | Rotate | Rotational rate command |

- **Control mode:** Field-centric drive
- **Deadband:** 10% on translation and rotation
- **Max angular rate:** 0.75 rotations/second

### Buttons and Triggers

| Input | Function | Type | Details |
|-------|----------|------|---------|
| **Start** | Reset heading | Press | Reseed field-centric heading |
| **Right Trigger (> 50%)** | Full close-shot sequence | Hold | Runs shooter + indexer close-shot sequence while held |
| **A** | Close shot preset | Press | Sets shooter to close-shot state |
| **X** | Far shot preset | Press | Sets shooter to far-shot state |
| **B** | Pass shot preset | Press | Sets shooter to pass-shot state |
| **POV Up** | Shooter eject | Press | Reverse/eject from shooter for 1 second |
| **Left Trigger (> 50%)** | Intake run | Hold | Runs intake command while held |
| **Left Bumper** | Intake jam clear | Press | Executes intake stop-jam action |
| **POV Right** | Climber extend | Hold | Extends climber arm while held |
| **POV Left** | Climber retract | Hold | Retracts climber arm while held |

### Currently Unbound / Disabled in Code

The following mappings are present as comments in `RobotContainer.java` and are **not active**:

- **A (driver):** Swerve brake mode
- **B (driver):** Point wheels to stick direction
- **POV Up/Down (driver drivetrain):** Slow robot-centric forward/backward drive
- **Right Bumper (driver):** Indexer feed
- **POV Down (driver):** Indexer eject
- **Climber preset extend/retract and stop bindings**
- **All SysId button combinations**

---

## Operator Controller (Port 1)

No active bindings are currently configured for the operator controller in `configureBindings()`.

---

## Driver Quick Map

```
LB: Intake jam clear                           RB: (unused)
LT: Intake run                                 RT: Full close-shot sequence

Left Stick: Translate                          Right Stick X: Rotate

                    POV Up: Shooter eject
        POV Left: Retract climber   POV Right: Extend climber
                  POV Down: (unused)

              X: Far shot      Y: (unused)

              A: Close shot    B: Pass shot
              Start: Reset heading
```

---

## Common Operations

### Shoot (close shot sequence)

1. Hold **Right Trigger** (>50%).
2. Sequence runs while held (close-shot RPM/angle plus feed behavior via command sequence).

### Set Shooter Presets

- **A**: Close shot preset
- **X**: Far shot preset
- **B**: Pass shot preset

### Intake

- Hold **Left Trigger** to run intake.
- Press **Left Bumper** to clear jams.

### Climber

- Hold **POV Right** to extend.
- Hold **POV Left** to retract.

---

## Troubleshooting

| Issue | Likely Cause | Solution |
|------|--------------|----------|
| Robot drives with wrong field orientation | Heading needs reseed | Press **Start** on driver controller |
| Shooter does not run sequence | Trigger under threshold | Hold **Right Trigger** past 50% |
| Intake does not run | Trigger under threshold | Hold **Left Trigger** past 50% |
| Operator inputs do nothing | No bindings on port 1 | Use driver controller bindings |

---

## Configuration Location

Controller bindings are configured in `src/main/java/frc/robot/RobotContainer.java`, inside `configureBindings()`.
