# 🌟 Star Wars Training Examples

These examples demonstrate the progression from simple subsystems to state machines.

**Start with HyperDrive.** Only move to LightsaberHilt when you understand why state machines help.

---

## Example 1: HyperDrive (Simple Approach)

**Files:** `HyperDrive.java`, `HyperDriveCommands.java`

This is how you should START any new subsystem. It uses:
- ✅ Boolean helper methods (`isReady()`, `isInHyperspace()`)
- ✅ Direct action methods (`charge()`, `engage()`, `disengage()`)
- ✅ One boolean to track state (`isEngaged`)
- ✅ Simple commands with `Commands.startEnd()` and `Commands.runOnce()`
- ✅ Sensor integration (`isMotivatorFunctional()`)

### When This Works Great
- Mechanism has 1-2 states to track
- States don't have complex transitions
- Boolean combinations are always valid

### Key Patterns to Notice

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

## Example 2: LightsaberHilt (State Machine)

**Files:** `LightsaberHilt.java`, `LightsaberHiltCommands.java`

This shows WHEN and WHY to upgrade to a state machine. It uses:
- ✅ State enum (`RETRACTED`, `EXTENDING`, `EXTENDED`, `RETRACTING`, `BLOCKED`)
- ✅ Sensor-driven transitions (contact sensor triggers BLOCKED)
- ✅ Position-based transitions (encoder position triggers state changes)
- ✅ Commands that REQUEST states and WAIT for completion
- ✅ Command sequences for complex behaviors

### When You Need This
- 3+ booleans tracking related state
- You're preventing "impossible" boolean combinations
- Transitions depend on sensors, timers, or other conditions
- You need to know "how did we get here?" for debugging

### Why Not Just Booleans?

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

### Key Patterns to Notice

**State machine in periodic():**
```java
@Override
public void periodic() {
    handleSensorTransitions();      // Sensors can force state changes
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

## The Progression

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

## Discussion Questions

After studying these examples, think about:

1. **HyperDrive only has one boolean (`isEngaged`).** What would make you add a state enum to it?

2. **LightsaberHilt has 5 states.** Could you reduce it to 3? What would you lose?

3. **The motivator sensor in HyperDrive** prevents charging if broken. Where is this checked - in the command or subsystem? Why?

4. **BLOCKED state in LightsaberHilt** is triggered by a sensor, not a request. Why is this handled differently than EXTENDING?

5. **Commands in LightsaberHiltCommands** use sequences and waits. Why can't HyperDriveCommands use `waitUntil(() -> hyperDrive.isReady())`? (Hint: what happens when charging stops?)

---

## Try It Yourself

1. **Add a state to HyperDrive**: What if the hyperdrive could "overheat" if engaged too long? Would you use a boolean or convert to an enum?

2. **Add a command to LightsaberHilt**: Create a "salute" command that extends, pauses, retracts twice quickly.

3. **Create your own**: Pick a Star Wars mechanism (deflector shield, tractor beam, droid launcher) and decide: simple or state machine?

---

*"Do. Or do not. There is no try."*
*— Yoda (on writing clean code)*
