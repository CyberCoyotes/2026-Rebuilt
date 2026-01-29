# Hardware Reference

A central location for motors, motor controllers, sensors, and CAN devices for quick reference.

---

## Swerve Drive

| Component | Type | Qty | Notes |
|-----------|------|-----|-------|
| Drive Motors | Kraken/Falcon + TalonFX | 4 | |
| Steer Motors | Kraken/Falcon + TalonFX | 4 | |
| Azimuth Encoders | CANcoder | 4 | encoders |
| IMU | Pigeon 2 | 1 | Gyro/accelerometer |
| LED Controller | CANdle | 1 | RGB LED strip control |
| CAN Bus | CANivore | 1 | CAN FD for drivetrain |

**CAN Bus:** `canivore`
**Configuration:** See `TunerConstants.java`

---

## Intake

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Rotator | Kraken X44 | TalonFX | 1 | Spins intake wheels |
| Slide  | Kraken X44 | TalonFX | 2 | (A, B) Pair extends/retracts intake |

### Intake Sensors

| Sensor | Type | Qty | Purpose |
|--------|------|-----|---------|
| Intake | CANrange ToF | 1 | Confirms fuel intake presence |

---

## Indexer

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Indexer | Kraken X60 | TalonFX | 1 | Feeds pieces to shooter |
| Conveyor | Kraken X44 | TalonFX | 1 | Moves pieces along hopper |

### Indexer Sensors

| Sensor | Type | Qty | Purpose |
|--------|------|-----|---------|
| Indexer | CANrange ToF | 1 | Detects fuel at indexer exit |
| Hopper Top | Grapple ToF | 3 | (A, B, C) Detects hopper fullness |

---

## Shooter

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Flywheel  | Falcon 500 | TalonFX | 3 | (A, B, C) 1 leader, 2 followers belted |
| Hood | Minion | TalonFXS | 1 | Adjusts shot angle |
| Counter Wheel | ---- | ---- | 0 | Counter-spin wheel |

---

## Climber

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Climber | Kraken X60 | TalonFX | 1 | Possible a two climb motor mechanism? |


---

## Vision

| Device | Type | Qty | Notes |
|--------|------|-----|-------|
| Limelight 3 | Camera | 1 | Network Tables interface |
| Limelight 4 | Camera | 1 | Network Tables interface |

---

## Other

| Device | Type | Qty | Notes |
|--------|------|-----|-------|
| MiniPDH | PDH | 2 | Power |
| Network Switch | Com | 1 | Power |
| Radio Power Mod | Com | 1 | Power |



## Hardware Summary

### Motors by Type

| Motor Type | Controller | Count | Subsystems |
|------------|------------|-------|------------|
| Kraken X60 | TalonFX | 5 | Climber (1), Drive (4) |
| Kraken X44 | TalonFX | 10 | Intake (3), Indexer (1), Shooter (1), Drive (4) |
| Falcon 500 | TalonFX | 3 | Shooter (3) |
| Minion | TalonFXS | 1 | Shooter |

**Total Motors:** 18

### Sensors by Type

| Sensor Type | Count | Subsystems |
|-------------|-------|------------|
| CANRange ToF | 5 | Intake (1), Indexer (4) |
| CANcoder | 4 | Swerve azimuth |
| Pigeon 2 | 1 | Navigation |
| Limelight | 2 | Vision |

---

## CAN Bus Assignment

| Bus | Subsystems |
|-----|------------|
| `canivore` | Swerve (motors, encoders, Pigeon, CANdle) |
| `rio` | Intake, Indexer, Shooter, Climber, ToF |

---

## Implementation Status

| Subsystem | Constants | Hardware IO | Status |
|-----------|-----------|-------------|--------|
| Swerve | TunerConstants.java | CommandSwerveDrivetrain | Complete |
| Intake | Complete | IntakeIOHardware.java | Partial (missing Slide B, CANrange) |
| Indexer | Needs update | IndexerIOHardware.java | Needs update (motor types, sensor types) |
| Shooter | Complete | ShooterIOHardware.java | Complete |
| Climber | Complete | ClimberIOHardware.java | Not implemented |
| Vision | Complete | VisionIOLimelight.java | Complete |

---

## Notes for Design Team

- Intake uses 3 motors total (1 rotator + 2 slide motors paired)
- Shooter flywheels run in leader-follower configuration (B and C follow A, belted)
- Indexer uses CANrange ToF for exit detection, Grapple ToF sensors for hopper fullness
- Hood uses smaller Minion motor with TalonFXS (not TalonFX)
- Swerve components are on separate CANivore bus for performance
