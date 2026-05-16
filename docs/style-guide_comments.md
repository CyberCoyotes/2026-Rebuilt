# Comment Style Guide

This guide defines how we write comments in Java files for this codebase.

The goals are:
- Make files easy to scan quickly
- Keep comments helpful without turning the code into a wall of text
- Stay readable on smaller screens
- Be consistent across subsystems, commands, `RobotContainer`, and `Constants`

## Core Rules

- Write comments to explain intent, behavior, and non-obvious decisions.
- Do not comment obvious syntax or restate what the code already says.
- Prefer fewer, better comments over many low-value comments.
- Keep comments accurate. If the code changes, update or remove the comment.
- Use plain ASCII by default.

## Section Headers

Use 3-line section headers for major sections only.

Example:

```java
// =====================================================================
// Driver Controller
// =====================================================================
```

Use this style for:
- major file sections
- subsystem structure blocks
- large configuration groupings
- major phases of a class

Examples:
- `IO Layer`
- `State`
- `Constructor`
- `Periodic`
- `Command Factories`
- `Vision Lookup Tables`

Do not use these for tiny local groupings.

## Mini-Section Comments

For small related blocks inside a major section, use a simple one-line comment.

Examples:

```java
// Roller
// Slide
// Target Setters
// Display Preset
```

Use mini-sections for:
- a few related methods
- a small cluster of fields
- a local grouping inside a larger section

Avoid decorative mini-headers like:

```java
// ==== Roller ====
// =====STATE MACHINE=====
// ====================
```

## Method Comments

Use Javadoc for public methods, commands, and APIs that other programmers will use.

Example:

```java
/**
 * Returns true if hood is at target pose within tolerance.
 */
public boolean isHoodAtPose() {
```

Use a normal `//` comment inside a method for brief local explanation.

Example:

```java
// Only periodic() earns the READY state; never set it directly.
```

## Inline Comments

Inline comments should be short and reserved for things that are not obvious.

Good:

```java
private ShotPreset displayPreset = null; // null = Vision mode
```

Bad:

```java
private int count = 0; // set count to zero
```

## TODO / FIXME

Use `TODO` when work remains but the code is safe enough to run for now.

Example:

```java
// TODO: Remove these debug publishers after chute tuning is finished.
```

Use `FIXME` when something is wrong, risky, or known-bad.

Example:

```java
// FIXME: Teleop speed is still scaled down for testing.
```

Rules:
- Be specific
- Name what needs to happen
- Avoid vague notes like `// TODO tune`

Better:

```java
// TODO: Verify hood tolerance on the rebuilt hood before next event.
```

## Block Comments

Use block comments sparingly for:
- hardware notes
- tuning procedures
- multi-line rationale
- safety notes

Example:

```java
/*
 * Followers must be set after bus optimization or the control link can break.
 */
```

Keep them tight. If a block comment starts reading like a document, move it to `/docs`.

## Constants File Guidance

In `Constants.java`:
- use major section headers for top-level groups like `Intake`, `Indexer`, `Flywheel`, `Hood`, `Vision`, `LEDs`, and `Auto`
- use mini-section comments for local groupings like `IDs`, `Slide setpoints and tuning values`, or `Vision-driven drivetrain rotation`
- keep tuning notes close to the values they describe
- prefer one clear comment above a group of related constants over repeating the same note on every line

## RobotContainer Guidance

In `RobotContainer.java`:
- use major section headers for `Controllers`, `Subsystems`, `Construction`, `Driver Controller`, `Operator Controller`, and `Public Accessors`
- use mini-section comments for optional or commented-out blocks like LED trigger ideas
- keep controller binding comments focused on what the binding does, not button trivia

## Subsystem Guidance

In subsystem classes:
- use major section headers for structure
- use mini-sections for grouped methods like `Roller`, `Slide`, `Target Setters`, or `Status Queries`
- comment state-machine transitions when the reason is non-obvious
- comment derived telemetry only when it helps future debugging

## Things To Avoid

- giant comment banners for every little block
- repeated comments that add no information
- stale comments that describe old behavior
- joke comments in production code if they reduce clarity
- inconsistent styles in the same file
- Unicode-heavy divider styles

## Preferred Pattern

Use this as the default:

```java
// =====================================================================
// Major Section
// =====================================================================

// Mini Section

/**
 * Public API description.
 */
public void doThing() {
    // Local explanation if needed.
}
```

## Final Rule

If a comment does not help a teammate understand:
- why something exists
- why it is written this way
- what needs to be tuned or fixed

then remove it.
