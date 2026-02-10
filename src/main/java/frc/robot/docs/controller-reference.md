# Controller Reference

Quick reference for Xbox controller bindings used to operate the robot.

---

## Controller Ports

| Port | Controller | Purpose |
|------|------------|---------|
| 0 | Driver | Drivetrain movement and field-centric controls |
| 1 | Operator | Mechanisms (shooter, intake, indexer, climber) |

---

## Driver Controller (Port 0)

Controls the drivetrain movement and field-centric orientation.

### Joysticks

| Input | Function | Details |
|-------|----------|---------|
| **Left Stick** | Translate | Forward/backward (Y-axis), Left/right (X-axis) |
| **Right Stick** | Rotate | Rotate robot (X-axis) |

- **Control Mode:** Field-centric drive (relative to field, not robot)
- **Deadband:** 10% on both translation and rotation
- **Max Speed:** 100% of robot top speed
- **Max Angular Rate:** 0.75 rotations per second

### Buttons

| Button | Function | Type | Details |
|--------|----------|------|---------|
| **A** | Brake | Hold | Lock wheels in X pattern (defensive position) |
| **B** | Point Wheels | Hold | Point wheels at left joystick direction (testing) |
| **Left Bumper** | Reset Heading | Press | Reset field-centric heading to current direction |

### D-Pad (POV)

| Direction | Function | Type | Details |
|-----------|----------|------|---------|
| **Up** | Slow Forward | Hold | Drive forward at 0.5 m/s (robot-centric) |
| **Down** | Slow Backward | Hold | Drive backward at 0.5 m/s (robot-centric) |

### SysId Routines (Advanced Testing)

| Combination | Function | Details |
|-------------|----------|---------|
| **Back + Y** | Dynamic Forward | SysId characterization routine |
| **Back + X** | Dynamic Reverse | SysId characterization routine |
| **Start + Y** | Quasistatic Forward | SysId characterization routine |
| **Start + X** | Quasistatic Reverse | SysId characterization routine |

> **Note:** SysId routines are for system identification and characterization. Only use during testing sessions.

---

## Operator Controller (Port 1)

Controls all robot mechanisms including shooter, intake, indexer, and climber.

### Shooter Controls

| Input | Function | Type | Details |
|-------|----------|------|---------|
| **Right Trigger** | Spin Up Shooter | Hold | Pre-rev shooter wheels (hold > 50%) |
| **Y** | Close Shot | Press | Prepare shooter for close-range shot |
| **X** | Far Shot | Press | Prepare shooter for far-range shot |
| **B** | Idle Shooter | Press | Stop shooter and return to idle |
| **A** | Full Shoot Sequence | Press | Complete close shot + feed + idle sequence |
| **POV Up** | Eject from Shooter | Press | Reverse shooter for 1 second (clear jams) |

### Indexer Controls

| Input | Function | Type | Details |
|-------|----------|------|---------|
| **Right Bumper** | Feed to Shooter | Hold | Feed game piece from indexer to shooter |
| **POV Down** | Eject from Indexer | Press | Reverse indexer for 1 second (clear jams) |

### Intake Controls

| Input | Function | Type | Details |
|-------|----------|------|---------|
| **Left Trigger** | Run Intake | Hold | Activate intake to collect game pieces (hold > 50%) |
| **Left Bumper** | Stop Jam | Press | Quick reverse to clear intake jams |

### Climber Controls

| Input | Function | Type | Details |
|-------|----------|------|---------|
| **POV Right** | Extend Arm | Hold | Extend climber arm |
| **POV Left** | Retract Arm | Hold | Retract climber arm |
| **Start** | Stop Climber | Press | Stop all climber movement |

---

## Button Layout Reference

### Xbox Controller Diagram

```
                    [View/Back]  [Menu/Start]
    
    [LB]                                        [RB]
    [LT]                                        [RT]
    
         (Left Stick)                  (Right Stick)
    
                       [POV/D-Pad]
                     
              [X]           [Y]
                  [Xbox]   
              [A]           [B]
```

### Driver (Port 0) Quick Map

```
LB: Reset Heading                              RB: (unused)
LT: (unused)                                   RT: (unused)

Left Stick: Translate                          Right Stick: Rotate

                    POV Up: Slow Forward
        POV Left:                  POV Right:
                  (unused)           (unused)
                  POV Down: Slow Backward

              X: (SysId)    Y: (SysId)
                      
              A: Brake      B: Point Wheels
```

### Operator (Port 1) Quick Map

```
LB: Stop Intake Jam                            RB: Feed to Shooter
LT: Run Intake                                 RT: Spin Up Shooter

(unused)                                       (unused)

                    POV Up: Eject from Shooter
        POV Left:                  POV Right:
        Retract Climber            Extend Climber
                  POV Down: Eject from Indexer

              X: Far Shot   Y: Close Shot
                      
              A: Shoot      B: Idle Shooter
              Sequence
```

---

## Common Operations

### Shooting Sequence (Manual)

1. **Right Trigger (hold)** → Spin up shooter wheels
2. Wait for shooter to reach target speed (LED indicator or dashboard)
3. **Y** or **X** → Set shooter angle (close or far)
4. Wait for angle adjustment complete (dashboard feedback)
5. **Right Bumper (hold)** → Feed game piece through indexer
6. **B** → Return shooter to idle when done

### Shooting Sequence (Automatic)

1. **A** → Execute full close shot sequence automatically
   - Spins up shooter
   - Sets close shot angle and speed
   - Feeds game piece when ready
   - Returns to idle

### Collecting Game Pieces

1. **Left Trigger (hold)** → Run intake
2. Release when game piece is secured
3. **Left Bumper** if jam occurs → Quick reverse

### Clearing Jams

- **Intake jam:** Left Bumper (quick reverse)
- **Indexer jam:** POV Down (1 second reverse)
- **Shooter jam:** POV Up (1 second reverse)

### Climbing Sequence

1. **POV Right (hold)** → Extend climber arm to bar
2. Release when positioned
3. **POV Left (hold)** → Retract to lift robot
4. **Start** → Stop climber when complete

---

## Troubleshooting

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| Robot drives wrong direction | Field-centric heading not reset | Press **Left Bumper** (driver) |
| Robot not responding | Wrong controller port | Verify controller is plugged into correct USB port |
| Intake won't stop | Button held down | Release **Left Trigger** |
| Shooter spinning but not feeding | Feed not activated | Hold **Right Bumper** to feed |
| Climber not moving | Climber stopped | Ensure **Start** hasn't been pressed |
| Drive feels "backward" | Facing wrong direction | Press **Left Bumper** to reset heading |

---

## Controller Testing Checklist

Before each match or practice session:

### Driver Controller (Port 0)
- [ ] Left stick translates robot in all directions
- [ ] Right stick rotates robot smoothly
- [ ] **A** button locks wheels in X pattern
- [ ] **Left Bumper** resets heading
- [ ] **POV Up/Down** slow drive works

### Operator Controller (Port 1)
- [ ] **Left Trigger** activates intake
- [ ] **Right Trigger** spins up shooter
- [ ] **Y/X** set shooter angles
- [ ] **Right Bumper** feeds indexer
- [ ] **A** runs full shoot sequence
- [ ] **POV Left/Right** move climber
- [ ] All eject buttons (POV Up/Down, Left Bumper) work

---

## Notes for Drivers

- **Field-centric drive:** Robot moves relative to field, not its own orientation
  - Push left stick forward → robot moves away from driver station
  - Even if robot is rotated 180°, forward stick still moves away from driver
- **Reset heading:** Press **Left Bumper** when robot is facing away from driver station
- **Brake mode:** Use **A** button for quick defensive positioning
- **Slow drive:** Use POV for precise alignment and docking

## Notes for Operators

- **Trigger thresholds:** Both triggers require > 50% press to activate
- **Hold vs Press:** Some buttons must be held (triggers, bumpers), others are single press (A/B/X/Y)
- **Eject functions:** All eject operations run for 1 second automatically
- **Shoot sequence:** Button **A** is the preferred method for standard shots
- **POV priority:** POV directions control different subsystems:
  - Up/Down: Shooter/Indexer eject
  - Left/Right: Climber movement

---

## Configuration Location

Controller bindings are configured in:
```
src/main/java/frc/robot/RobotContainer.java
```

See `configureBindings()` method for implementation details.

---

**Remember:** Controllers must be connected before robot code starts. Reconnecting during operation may cause unexpected behavior. If controllers are disconnected, disable and re-enable robot code.
