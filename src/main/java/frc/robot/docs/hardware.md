# Hardware Reference  

A central location for motors, motor controllers, sensors, and CAN devices for quick reference.

---

## Swerve Drive

| Component | Type | Qty | Notes |
|-----------|------|-----|-------|
| Drive Motors | Kraken X60 + TalonFX | 4 | |
| Steer Motors | Kraken X60 + TalonFX | 4 | |
| Azimuth Encoders | CANcoder | 4 | Absolute steering encoders |
| IMU | Pigeon 2 | 1 | Gyro / accelerometer |
| LED Controller | CANdle | 1 | RGB LED strip control |
| CAN Bus | CANivore | 1 | CAN FD for drivetrain |

**CAN Bus:** `canivore`  
**Configuration:** See `TunerConstants.java`

---

## Intake

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Rotator | Kraken X44 | TalonFX | 1 | Spins intake wheels |
| Slide  | Kraken X44 | TalonFX | 2 | (A, B) Paired extension/retraction |

### Intake Sensors

| Sensor | Type | Qty | Purpose |
|--------|------|-----|---------|
| Intake | CANrange ToF | 1 | Confirms fuel intake presence |

---

## Indexer / Hopper

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Indexer | Kraken X60 | TalonFX | 1 | Feeds pieces to shooter |
| Conveyor | Minion | TalonFXS | 1 | Hopper conveyor |

### Indexer / Hopper Sensors

| Sensor | Type | Qty | Purpose |
|--------|------|-----|---------|
| Indexer Exit | CANrange ToF | 1 | Detects fuel at indexer exit |
| Hopper Top | Grapple ToF | 3 | (A, B, C) Detects hopper fullness |

---

## Shooter

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Flywheel | Falcon 500 | TalonFX | 3 | (A, B, C) 1 leader, 2 followers, belted |
| Hood | Minion | TalonFXS | 1 | Hood angle adjustment |
| Counter Wheel | — | — | 0 | Not implemented |

### Hood Details

- **Motor:** Minion  
- **Gearbox:** Cycloidal  
- **Reduction:** 23:1  
- **Position Feedback:** WCP ThroughBore Encoder  
- **Control Mode:** Closed-loop position using external encoder  
- **Note:** Motor-integrated sensor not used for hood positioning

---

## Climber

| Component | Motor | Controller | Qty | Notes |
|-----------|-------|------------|-----|-------|
| Climber | Kraken X60 | TalonFX | 1 | Potential future dual-motor design |

---

## Vision

| Device | Type | Qty | Notes |
|--------|------|-----|-------|
| Limelight 3 | Camera | 1 | NetworkTables interface |
| Limelight 4 | Camera | 1 | NetworkTables interface |

---

## Other

| Device | Type | Qty | Notes |
|--------|------|-----|-------|
| MiniPDH | PDH | 2 | Power distribution |
| Network Switch | Com | 1 | Ethernet |
| Radio Power Module | Com | 1 | Radio power |

---

## Hardware Summary

### Motors by Type

| Motor Type | Controller | Count | Subsystems |
|------------|------------|-------|------------|
| Kraken X60 | TalonFX | 6 | Climber (1), Drive (4), Indexer (1) |
| Kraken X44 | TalonFX | 9 | Intake (3), Drive (4) |
| Falcon 500 | TalonFX | 3 | Shooter flywheels |
| Minion | TalonFXS | 2 | Shooter hood, Hopper conveyor |

**Total Motors:** 20

---

### Sensors by Type

| Sensor Type | Count | Subsystems |
|-------------|-------|------------|
| CANrange ToF | 2 | Intake (1), Indexer exit (1) |
| Grapple ToF | 3 | Hopper fullness |
| CANcoder | 4 | Swerve azimuth |
| WCP ThroughBore Encoder | 1 | Shooter hood |
| Pigeon 2 | 1 | Navigation |
| Limelight | 2 | Vision |

---

## CAN Bus Assignment

| Bus | Subsystems |
|-----|------------|
| `canivore` | Swerve motors, azimuth encoders, Pigeon, CANdle |
| `rio` | Intake, Indexer, Hopper conveyor, Shooter, Climber, ToF sensors |

---

## Implementation Status

| Subsystem | Constants | Hardware IO | Status |
|-----------|-----------|-------------|--------|
| Swerve | TunerConstants.java | CommandSwerveDrivetrain | Complete |
| Intake | Complete | IntakeIOHardware.java | Partial |
| Indexer / Hopper | Needs update | IndexerIOHardware.java | Needs update |
| Shooter | Complete | ShooterIOHardware.java | Complete |
| Climber | Complete | ClimberIOHardware.java | Not implemented |
| Vision | Complete | VisionIOLimelight.java | Complete |

---

## Notes for Design & Controls

- Intake uses 3 motors total (1 rotator + 2 paired slide motors)
- Shooter flywheels are leader–follower and mechanically belted
- Hopper conveyor uses Minion + TalonFXS
- Hood uses external ThroughBore encoder for accurate position control
- Swerve runs entirely on CANivore for performance isolation
