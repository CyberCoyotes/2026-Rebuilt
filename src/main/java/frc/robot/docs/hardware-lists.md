# Hardware Reference

A central location for motors, motor controllers, sensors, and CAN devices for quick reference.

---

## Swerve Drive

| Component | Motor | Controller | Qty |
|-----------|-------|------------|-----|
| Drive Motors | Kraken/Falcon | TalonFX | 4 |
| Steer Motors | Kraken/Falcon | TalonFX | 4 |
| Azimuth Encoders | CANcoder | - | 4 |
| IMU | Pigeon 2 | - | 1 |

**CAN Bus:** `canivore`
**Configuration:** See `TunerConstants.java`

---

## Intake

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Rotator | Kraken X44 | TalonFX | 1 | Spins intake wheels |
| Slide A | Kraken X44 | TalonFX | 1 | Extends/retracts intake |
| Slide B | Kraken X44 | TalonFX | 1 | Paired with Slide A |

### Intake Sensors

| Sensor | Type | Qty | Purpose |
|--------|------|-----|---------|
| Intake ToF | Playing With Fusion ToF | 1 | Confirms fuel presence |

---

## Indexer

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Indexer | Kraken X60 | TalonFX | 1 | Feeds pieces to shooter |
| Conveyor | Kraken X60 | TalonFX | 1 | Moves pieces along hopper |

### Indexer Sensors

| Sensor | Type | Qty | Purpose |
|--------|------|-----|---------|
| Indexer ToF | Playing With Fusion ToF | 1 | Detects fuel at indexer exit (optional) |
| Hopper ToF A | Playing With Fusion ToF | 1 | Hopper position A detection |
| Hopper ToF B | Playing With Fusion ToF | 1 | Hopper position B detection |
| Hopper ToF C | Playing With Fusion ToF | 1 | Hopper position C detection |

---

## Shooter

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Flywheel A | Falcon 500 | TalonFX | 1 | Leader motor |
| Flywheel B | Falcon 500 | TalonFX | 1 | Follower of A |
| Flywheel C | Falcon 500 | TalonFX | 1 | Follower of A |
| Hood | Minion | TalonFXS | 1 | Adjusts shot angle |
| Counter Wheel | Kraken X44 | TalonFX | 1 | Counter-spin wheel |

---

## Climber

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Climb A | Kraken X60 | TalonFX | 1 | Climb mechanism |
| Climb B | Kraken X60 | TalonFX | 1 | Climb mechanism |

---

## Vision

| Device | Type | Qty | Notes |
|--------|------|-----|-------|
| Limelight 3 | Camera | 1 | Network Tables interface |
| Limelight 4 | Camera | 1 | Network Tables interface |

---

## Hardware Summary

### Motors by Type

| Motor Type | Controller | Count | Subsystems |
|------------|------------|-------|------------|
| Kraken X60 | TalonFX | 4 | Indexer (2), Climber (2) |
| Kraken X44 | TalonFX | 4 | Intake (3), Shooter (1) |
| Falcon 500 | TalonFX | 3 | Shooter flywheels |
| Minion | TalonFXS | 1 | Shooter hood |
| Kraken/Falcon | TalonFX | 8 | Swerve drive |

**Total Motors:** 20

### Sensors by Type

| Sensor Type | Count | Subsystems |
|-------------|-------|------------|
| Playing With Fusion ToF | 5 | Intake (1), Indexer (4) |
| CANcoder | 4 | Swerve azimuth |
| Pigeon 2 | 1 | Navigation |
| Limelight | 2 | Vision |

---

## CAN Bus Assignment

| Bus | Subsystems |
|-----|------------|
| `canivore` | Swerve (motors, encoders, Pigeon) |
| `rio` | Intake, Indexer, Shooter, Climber |

---

## Implementation Status

| Subsystem | Constants | Hardware IO | Status |
|-----------|-----------|-------------|--------|
| Swerve | TunerConstants.java | CommandSwerveDrivetrain | Complete |
| Intake | Complete | IntakeIOHardware.java | Partial (missing Slide B, ToF) |
| Indexer | Complete | IndexerIOHardware.java | Complete |
| Shooter | Complete | ShooterIOHardware.java | Complete |
| Climber | Complete | ClimberIOHardware.java | Not implemented |
| Vision | Complete | VisionIOLimelight.java | Complete |

---

## Notes for Design Team

- Intake uses 3 motors total (1 rotator + 2 slide motors working together)
- Shooter flywheels run in leader-follower configuration (B and C follow A)
- All ToF sensors are Playing With Fusion brand, configured for short-range detection
- Hood uses smaller Minion motor with TalonFXS (not TalonFX)
- Swerve components are on separate CANivore bus for performance
