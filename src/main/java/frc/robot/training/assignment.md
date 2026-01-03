# 🚀 Practice Subsystem Assignment

## Overview

Create a **fictional subsystem** for a robot mechanism that doesn't exist. This lets you practice the patterns without worrying about "getting it wrong" for real hardware.

**Time:** 1-2 hours  
**Due:** Before kickoff (January 10, 2026)  
**Submit:** Pull request to `training-main` branch

---

## Requirements

### Part 1: Basic Subsystem (Required)

Create a subsystem class that includes:

| Requirement | Details |
|------------|---------|
| **Extends SubsystemBase** | Standard FRC subsystem |
| **At least one motor** | Use `TalonFX` with a fake CAN ID (50-99) |
| **At least one sensor** | DigitalInput, analog sensor, or motor's built-in encoder |
| **Constructor** | Initialize all hardware |
| **Action methods** | At least 3 methods that do something (run, stop, reverse, etc.) |
| **Sensor method** | At least 1 method that returns sensor data (hasPiece, getPosition, etc.) |
| **periodic()** | Even if empty, include it with a comment about what could go here |
| **Constants** | Use constants for speeds, IDs — can be in the same file for practice |

### Part 2: Commands (Required)

Create a commands class with:

| Requirement | Details |
|------------|---------|
| **Factory pattern** | `public class YourCommands` with static methods |
| **Private constructor** | Prevent instantiation |
| **At least 2 commands** | Using `Commands.startEnd()`, `Commands.run()`, or `Commands.runOnce()` |
| **Subsystem requirement** | Pass subsystem to `Commands.xxx(..., subsystem)` |

### Part 3: Stretch Goals (Optional)

If you finish early, try adding:

- [ ] **State enum** — Track IDLE, ACTIVE, etc.
- [ ] **Automatic transitions** — If sensor triggers, change state
- [ ] **Logging** — Print state changes to console
- [ ] **Second motor or servo** — Coordinated movement
- [ ] **Simulation support** — Different behavior when `RobotBase.isSimulation()`

---

## Example Ideas

Pick something fun! Here are some ideas:

| Name | Description | Sensors |
|------|-------------|---------|
| CandyLauncher | Fires candy at mentors | Beam break for candy loaded |
| PoolNoodleYeeter | Launches pool noodles | Limit switch for arm position |
| BubbleMachine | Makes bubbles aggressively | Time-based (no sensor needed) |
| ConfettiCannon | Celebration mechanism | Pressure sensor |
| PizzaSpinner | Spins pizza dough | Encoder for rotation count |
| SockSorter | Sorts socks by color | Color sensor |
| TreatDispenser | Gives dog treats | IR distance sensor |

---

## File Template

Use this as your starting point:

### Subsystem File: `YourSubsystem_YourName.java`

```java
package frc.robot.training;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class CandyLauncher_YourName extends SubsystemBase {
    
    // ========== CONSTANTS ==========
    // Use IDs 50-99 for practice (won't conflict with real robot)
    private static final int LAUNCHER_MOTOR_ID = 50;
    private static final int BEAM_BREAK_PORT = 0;
    
    private static final double LAUNCH_SPEED = 1.0;
    private static final double INTAKE_SPEED = 0.3;
    
    // ========== HARDWARE ==========
    private final TalonFX launcherMotor;
    private final DigitalInput beamBreak;
    
    // ========== CONSTRUCTOR ==========
    public CandyLauncher_YourName() {
        launcherMotor = new TalonFX(LAUNCHER_MOTOR_ID);
        beamBreak = new DigitalInput(BEAM_BREAK_PORT);
        
        // Configure motor settings here
        // (We'll learn more about this with real hardware)
    }
    
    // ========== ACTION METHODS ==========
    
    /** Run the launcher at full speed to fire candy */
    public void launch() {
        launcherMotor.set(LAUNCH_SPEED);
    }
    
    /** Run slowly to intake a piece of candy */
    public void intake() {
        launcherMotor.set(INTAKE_SPEED);
    }
    
    /** Stop the launcher motor */
    public void stop() {
        launcherMotor.set(0);
    }
    
    // ========== SENSOR METHODS ==========
    
    /** 
     * Check if candy is loaded and ready to fire.
     * @return true if beam break is triggered (candy present)
     */
    public boolean hasCandyLoaded() {
        // DigitalInput.get() returns true when NOT blocked
        // So we invert it: blocked = candy present
        return !beamBreak.get();
    }
    
    // ========== PERIODIC ==========
    
    @Override
    public void periodic() {
        // This runs every 20ms
        // Could add:
        // - Logging current state
        // - Updating SmartDashboard
        // - State machine logic
    }
}
```

### Commands File: `YourSubsystemCommands_YourName.java`

```java
package frc.robot.training;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CandyLauncherCommands_YourName {
    
    // Private constructor prevents instantiation
    // This class only has static factory methods
    private CandyLauncherCommands_YourName() {}
    
    /**
     * Command to intake candy until loaded.
     * Runs intake, automatically stops when candy detected.
     */
    public static Command intakeCandy(CandyLauncher_YourName launcher) {
        return Commands.startEnd(
            () -> launcher.intake(),      // Start: begin intaking
            () -> launcher.stop(),        // End: stop motor
            launcher                      // Requires this subsystem
        ).until(() -> launcher.hasCandyLoaded());  // Stop when candy loaded
    }
    
    /**
     * Command to launch candy.
     * Runs while button is held, stops when released.
     */
    public static Command launchCandy(CandyLauncher_YourName launcher) {
        return Commands.startEnd(
            () -> launcher.launch(),      // Start: fire!
            () -> launcher.stop(),        // End: stop motor
            launcher                      // Requires this subsystem
        );
    }
    
    /**
     * Command to launch only if candy is loaded.
     * Does nothing if no candy present.
     */
    public static Command launchIfReady(CandyLauncher_YourName launcher) {
        return Commands.either(
            launchCandy(launcher),              // If candy loaded, launch
            Commands.none(),                     // If no candy, do nothing
            () -> launcher.hasCandyLoaded()     // Condition to check
        );
    }
}
```

---

## Pull Request Template

When you open your PR, use this format:

```markdown
## What did I build?
[Describe your fictional mechanism in 1-2 sentences]

## What does it do?
- [Action 1]
- [Action 2]
- [Sensor reading]

## Files Added
- [ ] Subsystem class
- [ ] Commands class

## Checklist
- [ ] Code compiles (no red squiggles)
- [ ] Used fake CAN IDs (50-99)
- [ ] Followed naming conventions
- [ ] Added comments explaining what things do
- [ ] Constants are not magic numbers in the code

## Questions for Reviewers
[Anything you're unsure about? Ask here!]
```

---

## Grading Rubric

This isn't graded for points, but here's what we're looking for:

| Criteria | What Good Looks Like |
|----------|---------------------|
| **Compiles** | No errors, imports are correct |
| **Structure** | Follows the template organization |
| **Naming** | Clear method names, constants named descriptively |
| **Comments** | Explains the "why" not just the "what" |
| **Commands** | Uses factory pattern correctly, requires subsystem |
| **Git** | Multiple commits with good messages, proper PR |

---

## Getting Help

**Stuck on Git?**
- Check the handbook Git section
- Ask in Slack #programming channel
- Pair up with someone who's done it

**Stuck on code?**
- Look at the templates in this file
- Check the handbook Level 3/4 sections
- Ask Claude (but make sure you understand the answer!)

**Something broken?**
- Post in Slack with:
  - What you tried
  - What happened
  - What you expected

---

## After You're Done

Once your PR is merged:

1. ✅ You're ready for kickoff Git workflow
2. ✅ You understand subsystem structure
3. ✅ You can build real subsystems using these patterns
4. ✅ You've practiced code review

The training folder gets deleted, but **the knowledge stays with you!**

---

*Good luck, and make something fun! 🎉*
