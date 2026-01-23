# Shooter Subsystem - Student Learning Tasks

## Overview

Your initial shooter implementation (`ShooterSubsystemBasics.java`) has been preserved as a learning reference. The production code now uses an advanced IO-layered architecture in `ShooterSubsystem.java`. This document outlines valuable tasks you can work on to learn FRC software development while contributing to competition readiness.

## Architecture Comparison

### Your Implementation (ShooterSubsystemBasics)
- **Direct hardware control**: TalonFX motors instantiated directly in subsystem
- **Simple state machine**: IDLE → PREPARING → SHOOTING → EJECTING
- **Easy to understand**: Great for learning basic motor control

### Production Implementation (ShooterSubsystem)
- **IO-layered architecture**: Hardware abstraction through ShooterIO interface
- **Testable & replayable**: Can run in simulation or replay logs without hardware
- **Feedback-based transitions**: State changes verify motors reached targets
- **Vision integration**: Calculates shot parameters from distance
- **Comprehensive diagnostics**: Monitoring for overheating, overcurrent, sync issues

**Learning Goal**: Understand why top teams (254, 1678, 6328) use this pattern

---

## Priority Tasks

### P0: Hardware Calibration (Critical for Competition)

#### Task 1.1: Measure Gear Ratios
**Location**: `TalonFXConfigs.java`
**Current State**: Placeholder values (1.5 for flywheels, 100 for hood)
**What to Do**:
1. Count teeth on flywheel gears: motor pinion → intermediate gear → flywheel gear
2. Calculate ratio: (intermediate/pinion) × (flywheel/intermediate)
3. Update `FLYWHEEL_GEAR_RATIO` in TalonFXConfigs.java
4. Repeat for hood mechanism
5. Document the physical gear counts in comments

**Success Criteria**:
- Robot code reports correct RPM matching physical measurements
- Hood angle in degrees matches protractor measurements

**Estimated Time**: 30-45 minutes with robot access

---

#### Task 1.2: Tune PID Gains
**Location**: `TalonFXConfigs.java` (lines with kP, kI, kD)
**Current State**: Untuned placeholder gains
**What to Do**:
1. **Flywheel velocity control**:
   - Start with current kP = 0.1
   - Command shooter to 3000 RPM using ShooterCommands.spinUp()
   - Use Phoenix Tuner X or AdvantageScope to plot velocity vs target
   - Increase kP until oscillation, then reduce by 50%
   - Add kD if needed to reduce overshoot
   - Document final gains in code comments

2. **Hood position control**:
   - Start with current kP = 1.0
   - Command hood to 45 degrees
   - Use same tuning process
   - Ensure no mechanical binding

**Tools Needed**:
- Phoenix Tuner X for live plotting
- AdvantageScope for log analysis
- Robot on blocks (flywheels) or safe test area

**Success Criteria**:
- Flywheel reaches target RPM within 0.5 seconds with <2% overshoot
- Hood reaches target angle within 0.5 seconds with <1° overshoot

**Estimated Time**: 1-2 hours on hardware

---

#### Task 1.3: Calibrate Hood Soft Limits
**Location**: `TalonFXConfigs.java` (hoodConfig soft limits - currently disabled)
**What to Do**:
1. Manually move hood to minimum safe angle (watch for collisions!)
2. Read position from Phoenix Tuner X (in rotations)
3. Manually move hood to maximum safe angle
4. Read position from Phoenix Tuner X
5. Enable soft limits in code:
   ```java
   hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
   hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POSITION_ROTATIONS;
   hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
   hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_POSITION_ROTATIONS;
   ```
6. Test that hood stops at limits (CAREFULLY!)

**Safety**: Have someone ready to disable robot if hood moves unsafely

**Success Criteria**:
- Hood cannot be commanded past physical limits
- Soft limits prevent mechanical damage

**Estimated Time**: 30 minutes

---

### P1: Testing & Validation

#### Task 2.1: Create Unit Tests
**Location**: Create `src/test/java/frc/robot/subsystems/ShooterSubsystemTest.java`
**What to Do**:
1. Test state machine transitions:
   ```java
   @Test
   void testIdleToSpinupTransition() {
       ShooterSubsystem shooter = new ShooterSubsystem(new ShooterIOSim());
       shooter.setTargetVelocity(3000);
       shooter.spinup();
       assertEquals(ShooterState.SPINUP, shooter.getState());
   }
   ```
2. Test preset configurations (closeShot(), farShot(), pass())
3. Test vision distance calculations
4. Test diagnostic methods (isOverheating(), isOverCurrent())

**Learning Goal**: Understand test-driven development and simulation testing

**Success Criteria**:
- At least 10 unit tests covering major functionality
- All tests pass with `./gradlew test`

**Estimated Time**: 2-3 hours

---

#### Task 2.2: Measure Current Draws
**Location**: Document in comments in `TalonFXConfigs.java`
**What to Do**:
1. Run shooter at full speed with game pieces
2. Monitor current draw in Phoenix Tuner X or Driver Station
3. Record peak current for:
   - Single shot
   - Continuous shooting (5+ pieces)
   - Hood movement under load
4. Verify current limits in TalonFXConfigs are appropriate
5. Document findings: "Measured peak: X amps during [condition]"

**Success Criteria**:
- Current limits prevent breaker trips
- Limits high enough to allow full performance
- Documentation helps future students understand choices

**Estimated Time**: 30 minutes

---

### P2: Integration & Features

#### Task 3.1: Add Telemetry Publishing
**Location**: `ShooterSubsystem.java` periodic() method
**Current State**: Uses SmartDashboard (not compatible with Elastic)
**What to Do**:
1. Study how other subsystems publish to NetworkTables for Elastic
2. Replace SmartDashboard calls with NetworkTables publishing:
   ```java
   NetworkTableInstance.getDefault()
       .getTable("Shooter")
       .getEntry("State")
       .setString(currentState.toString());
   ```
3. Add telemetry for:
   - Current state
   - Target vs actual velocities
   - Target vs actual hood angle
   - isReady() status
   - Diagnostic flags (overheating, overcurrent)

**Learning Goal**: Understand telemetry architecture for dashboard visualization

**Success Criteria**:
- All shooter data visible in Elastic dashboard
- No SmartDashboard dependencies remain

**Estimated Time**: 1-2 hours

---

#### Task 3.2: Integrate Shooter into RobotContainer
**Location**: `RobotContainer.java`
**What to Do**:
1. Instantiate ShooterSubsystem in RobotContainer:
   ```java
   private final ShooterSubsystem shooter;

   public RobotContainer() {
       // In constructor
       if (Robot.isReal()) {
           shooter = new ShooterSubsystem(new ShooterIOTalonFX());
       } else {
           shooter = new ShooterSubsystem(new ShooterIOSim());
       }

       configureBindings();
   }
   ```

2. Add button bindings in configureBindings():
   ```java
   // Example bindings
   joystick.rightBumper().whileTrue(ShooterCommands.closeShot(shooter));
   joystick.leftTrigger().whileTrue(ShooterCommands.farShot(shooter));
   joystick.rightTrigger().whileTrue(ShooterCommands.visionShot(shooter, vision));
   joystick.x().onTrue(ShooterCommands.idle(shooter));
   joystick.y().whileTrue(ShooterCommands.eject(shooter, 2.0));
   ```

**Success Criteria**:
- Shooter can be controlled from Xbox controller
- Commands work in both simulation and on hardware

**Estimated Time**: 1 hour

---

#### Task 3.3: Implement Ballistic Calculations
**Location**: `ShooterSubsystem.java` updateFromDistance() method (line 312)
**Current State**: Simple linear interpolation (placeholder)
**What to Do**:
1. Research projectile motion physics
2. Implement proper ballistic trajectory calculation:
   - Input: distance to target, target height
   - Output: required velocity and angle
   - Consider: gravity, air resistance (optional), wheel diameter
3. Test with actual shooting and adjust for real-world effects

**Learning Goal**: Apply physics to robotics, iterative tuning

**Success Criteria**:
- Vision-based shots are more accurate than presets
- Can hit targets at varying distances

**Estimated Time**: 3-4 hours (includes research and testing)

---

### P3: Documentation & Learning

#### Task 4.1: Document Architecture Differences
**Location**: Create `docs/shooter-architecture.md`
**What to Do**:
1. Write a comparison document explaining:
   - Why IO-layered architecture is used
   - Benefits: testability, simulation, log replay
   - Trade-offs: more files, steeper learning curve
   - When to use each approach
2. Include code snippets from both versions
3. Explain how ShooterIO, ShooterIOTalonFX, and ShooterIOSim work together

**Learning Goal**: Deepen understanding by teaching others

**Success Criteria**:
- Document is clear enough for a new student to understand
- Covers key architectural patterns

**Estimated Time**: 2 hours

---

#### Task 4.2: Add Tuning Widgets
**Location**: `ShooterSubsystem.java` or new ShooterTuner.java
**What to Do**:
1. Add NetworkTables entries for live tuning:
   ```java
   SmartDashboard.putNumber("Shooter/TuneRPM", CLOSE_SHOT_RPM);
   SmartDashboard.putNumber("Shooter/TuneAngle", CLOSE_SHOT_ANGLE);
   ```
2. Read tuning values in periodic() and apply them:
   ```java
   double tuneRPM = SmartDashboard.getNumber("Shooter/TuneRPM", targetFlywheelRPM);
   ```
3. Add button to test current tuning values

**Learning Goal**: Understand iterative tuning workflow

**Success Criteria**:
- Can adjust shot parameters from dashboard without redeploying code
- Changes take effect immediately

**Estimated Time**: 1 hour

---

## Task Assignment Recommendations

**Week 1**: Focus on P0 tasks (hardware calibration)
- Measure gear ratios
- Tune PID gains
- Calibrate soft limits

**Week 2**: P1 tasks (testing & validation)
- Write unit tests
- Measure current draws

**Week 3**: P2 tasks (integration)
- Integrate into RobotContainer
- Add telemetry
- Implement ballistics (if time permits)

**Ongoing**: P3 tasks (documentation)
- Complete during downtime or when robot unavailable

---

## Getting Help

**When Stuck**:
1. Check AdvantageKit examples: https://github.com/Mechanical-Advantage/AdvantageKit
2. Review CTRE Phoenix 6 documentation: https://v6.docs.ctr-electronics.com
3. Study top team code: FRC 254, 1678, 6328 on GitHub
4. Ask mentors or experienced team members

**Code Review**:
- Submit changes via pull request
- Explain what you changed and why
- Include test results or videos of robot behavior

---

## Success Metrics

By completing these tasks, you will:
- ✅ Understand IO-layered architecture (industry pattern)
- ✅ Learn PID tuning (critical FRC skill)
- ✅ Practice test-driven development
- ✅ Gain experience with telemetry systems
- ✅ Apply physics to real-world robotics
- ✅ Contribute valuable work to competition robot

**Most Importantly**: You'll see how your foundational work (the IO layer) became the backbone of the production system. The architecture you helped build is what makes all these advanced features possible.

---

## Notes

- **ShooterSubsystemBasics.java**: Your original work is preserved. Feel free to reference or modify it for learning.
- **Practice Robot**: If available, you can test on practice robot while competition robot is in use.
- **Safety First**: Always have supervision when running motors at speed.
- **Ask Questions**: No question is too basic. The goal is learning.

Good luck! 🚀
