# 🚀 Practice Subsystem Assignment

**This folder is temporary.** It exists for pre-season Git and subsystem practice. Everything here will be deleted after kickoff when we build real subsystems.

---

## Purpose

This assignment lets you practice:
1. **Git workflow** — branching, committing, pushing, pull requests, issues
2. **Subsystem structure** — how FRC subsystems are organized
3. **Command patterns** — how commands interact with subsystems
4. **Code review** — giving and receiving feedback

---

## Rules

### ✅ DO
- Create your own branch: `yourname-subsystem`
- Make your subsystem something fun (HyperDrive, LightsaberHilt, PoolNoodleYeeter, etc.)
- Use fake CAN IDs (pick numbers 50-99 to avoid conflicts)
- Follow the same patterns as the examples
- Ask questions in your PR!

### ❌ DON'T
- Merge directly to `main` — always use a PR
- Use real CAN IDs (those are reserved for actual hardware)
- Worry about perfect code — this is for learning
- Skip the PR process "because it's just practice"

---

## Git Workflow

```
1. Create your branch
   git checkout -b yourname-hyperdrive

2. Create your files in this folder
   training/
   ├── HyperDrive_YourName.java        (your subsystem)
   └── HyperDriveCommands_YourName.java (your commands)

3. Commit often with good messages
   git add .
   git commit -m "Add basic motor control to HyperDrive"

4. Push your branch
   git push -u origin yourname-hyperdrive

5. Open a Pull Request on GitHub
   - Base: training-main (NOT main!)
   - Compare: your branch
   - Fill out the PR template (see below)

6. Get code review, make changes, get approved

7. Merge your PR (squash and merge)
```

---

## Requirements

### Part 1: Basic Subsystem (Required)

Create a subsystem class that includes:

| Requirement | Details |
|-------------|---------|
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
|-------------|---------|
| **Factory pattern** | `public class YourCommands` with static methods |
| **Private constructor** | Prevent instantiation |
| **At least 2 commands** | Using `Commands.startEnd()`, `Commands.run()`, or `Commands.runOnce()` |
| **Subsystem requirement** | Pass subsystem to `Commands.xxx(..., subsystem)` |

### Part 3: Stretch Goals (Optional)

If you finish early, try adding:

- [ ] **State enum** — Track IDLE, ACTIVE, etc. (see LightsaberHilt example)
- [ ] **Automatic transitions** — If sensor triggers, change state
- [ ] **Logging** — Print state changes to console
- [ ] **Second motor or servo** — Coordinated movement
- [ ] **Simulation support** — Different behavior when `RobotBase.isSimulation()`

---

## Star Wars Examples

We've provided two complete examples in the `examples/` folder. **Study these before starting your own.**

### Example 1: HyperDrive (Simple Approach)

**Files:** `examples/HyperDrive.java`, `examples/HyperDriveCommands.java`

This is how you should START any new subsystem. It uses:
- ✅ Boolean helper methods (`isReady()`, `isInHyperspace()`)
- ✅ Direct action methods (`charge()`, `engage()`, `disengage()`)
- ✅ One boolean to track state (`isEngaged`)
- ✅ Simple commands with `Commands.startEnd()` and `Commands.runOnce()`
- ✅ Sensor integration (`isMotivatorFunctional()`)

#### When This Works Great
- Mechanism has 1-2 states to track
- States don't have complex transitions
- Boolean combinations are always valid

#### Key Patterns to Notice

**Helper methods that read clearly:**
```java
if (isReady() && isMotivatorFunctional()) {
    // This reads like English!
}
```

**Commands that handle cleanup automatically:**
```java
return Commands.startEnd(
    () -> hyperDrive.charge(),    // Runs when button pressed
    () -> hyperDrive.disengage(), // Runs when button released
    hyperDrive
);
```

---

### Example 2: LightsaberHilt (State Machine)

**Files:** `examples/LightsaberHilt.java`, `examples/LightsaberHiltCommands.java`

This shows WHEN and WHY to upgrade to a state machine. It uses:
- ✅ State enum (`RETRACTED`, `EXTENDING`, `EXTENDED`, `RETRACTING`, `BLOCKED`)
- ✅ Sensor-driven transitions (contact sensor triggers BLOCKED)
- ✅ Position-based transitions (encoder position triggers state changes)
- ✅ Commands that REQUEST states and WAIT for completion
- ✅ Command sequences for complex behaviors

#### When You Need This
- 3+ booleans tracking related state
- You're preventing "impossible" boolean combinations
- Transitions depend on sensors, timers, or other conditions
- You need to know "how did we get here?" for debugging

#### Why Not Just Booleans?

Imagine tracking the lightsaber with booleans:
```java
// DANGER: What if isExtending AND isRetracted are both true?
private boolean isExtending = false;
private boolean isRetracting = false;  
private boolean isExtended = false;
private boolean isRetracted = true;
private boolean isBlocked = false;
```

With an enum, this is impossible:
```java
// SAFE: Always exactly ONE state
private State currentState = State.RETRACTED;
```

#### Key Patterns to Notice

**State machine in periodic():**
```java
@Override
public void periodic() {
    handleSensorTransitions();       // Sensors can force state changes
    handleDesiredStateTransitions(); // Move toward requested state
    executeCurrentStateBehavior();   // Run motors based on state
}
```

**Commands request states, not motor speeds:**
```java
// Command just requests - state machine handles the rest
Commands.runOnce(() -> saber.requestIgnite(), saber)
```

**Waiting for state completion:**
```java
Commands.sequence(
    Commands.runOnce(() -> saber.requestIgnite(), saber),
    Commands.waitUntil(() -> saber.isReadyForCombat())
)
```

---

### The Progression

```
┌─────────────────────────────────────────────────────────────┐
│  SIMPLE (HyperDrive)                                        │
│  • 1-2 booleans                                             │
│  • Direct action methods                                    │
│  • Commands call methods directly                           │
│                                                             │
│  START HERE for every new subsystem!                        │
└─────────────────────────────────────────────────────────────┘
                          │
                          │ When you notice:
                          │ • 3+ booleans
                          │ • Impossible state bugs
                          │ • Confusing transitions
                          ▼
┌─────────────────────────────────────────────────────────────┐
│  STATE MACHINE (LightsaberHilt)                             │
│  • State enum                                               │
│  • Request/transition pattern                               │
│  • Commands wait for states                                 │
│                                                             │
│  Only upgrade when simple becomes painful!                  │
└─────────────────────────────────────────────────────────────┘
```

---

## File Naming

Put your name or initials in your filename to avoid conflicts:

```
training/
├── ASSIGNMENT.md                        (this file)
├── README.md                            (points here)
├── HyperDrive_Chewie.java      
├── HyperDriveCommands_Chewie.java
├── LightsaberHilt_R2D2.java
├── LightsaberHiltCommands_R2D2.java
└── examples/                            (reference examples)
    ├── HyperDrive.java
    ├── HyperDriveCommands.java
    ├── LightsaberHilt.java
    └── LightsaberHiltCommands.java
```

---

## Pull Request Template

When you open your PR, use this format:

```markdown
## What Did I Build?
[Describe your fictional mechanism in 1-2 sentences]

## What Does It Do?
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

## What We're Looking For

This isn't graded for points, but here's what good looks like:

| Criteria | What Good Looks Like |
|----------|----------------------|
| **Compiles** | No errors, imports are correct |
| **Structure** | Follows the template organization |
| **Naming** | Clear method names, constants named descriptively |
| **Comments** | Explains the "why" not just the "what" |
| **Commands** | Uses factory pattern correctly, requires subsystem |
| **Git** | Multiple commits with good messages, proper PR |

---

## Discussion Questions

After studying the examples and completing your subsystem, think about:

1. **HyperDrive only has one boolean (`isEngaged`).** What would make you add a state enum to it?

2. **LightsaberHilt has 5 states.** Could you reduce it to 3? What would you lose?

3. **The motivator sensor in HyperDrive** prevents charging if broken. Where is this checked — in the command or subsystem? Why?

4. **BLOCKED state in LightsaberHilt** is triggered by a sensor, not a request. Why is this handled differently than EXTENDING?

5. **Commands in LightsaberHiltCommands** use sequences and waits. Why can't HyperDriveCommands use `waitUntil(() -> hyperDrive.isReady())`? (Hint: what happens when charging stops?)

---

## Getting Help

**Stuck on Git?**
- Check the handbook Git section
- Ask in person or via email
- Pair up with someone who's done it

**Stuck on code?**
- Look at the examples in `examples/`
- Check the handbook Level 3/4 sections
- Ask Claude (but make sure you understand the answer!)

**Something broken?**
- Post in Slack with:
  - What you tried
  - What happened
  - What you expected

---

## After Kickoff

This entire folder gets deleted (or archived for reference). Your real subsystems will live in:
```
src/main/java/frc/robot/subsystems/
src/main/java/frc/robot/commands/
```

The patterns you practice here transfer directly — just with real mechanisms and real CAN IDs.

Once your PR is merged:
- ✅ You're ready for kickoff Git workflow
- ✅ You understand subsystem structure
- ✅ You can build real subsystems using these patterns
- ✅ You've practiced code review

**The training folder gets deleted, but the knowledge stays with you!**

---

*"Do. Or do not. There is no try."*
*— Yoda (on writing clean code)*
