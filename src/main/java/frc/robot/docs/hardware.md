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
| Slide  | Kraken X44 | TalonFX | 1 | Extension/retraction |

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
| Hopper Top | CANrange ToF | 3 | (A, B, C) Detects hopper fullness |

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
| CANrange ToF | 5 | Intake (1), Indexer (1), Hopper (3) |
| CANcoder | 4 | Swerve azimuth |
| CANcoder (ThroughBore) | 1 | Shooter hood |
| Pigeon 2 | 1 | Navigation |
| Limelight | 2 | Vision |


---

## CAN Bus Assignment

| Bus | Subsystems |
|-----|------------|
| `canivore` | Swerve motors, azimuth encoders, Pigeon, CANdle |
| `rio` | Intake, Indexer, Hopper conveyor, Shooter, Climber, ToF sensors |

---

## Hypothetical Single-PDH Allocation (Capacity Check)

This section models the robot using **one CTRE Mini PDH** to validate channel capacity.
This configuration is **not viable**, but is documented to justify the use of multiple PDHs.

### Assumptions
- One **Mini PDH** with **24 total channels**
- All motor controllers require individual PDH channels
- RoboRIO requires a PDH channel
- Mini Power Modules require PDH channels
- All CAN devices and Limelights are powered from Mini Power Modules

---

### Single PDH — All Loads Combined

#### Motors

| Channel | Load |
|--------:|------|
| 1–4 | Swerve Drive Motors (4× Kraken X60) |
| 5–8 | Swerve Steer Motors (4× Kraken X60) |
| 9 | Intake Rotator (Kraken X44) |
| 10 | Intake Slide Motor (Kraken X44) |
| 11 | Indexer (Kraken X60) |
| 12 | Hopper Conveyor (Minion) |
| 13–15 | Shooter Flywheels (3× Falcon 500) |
| 16 | Shooter Hood (Minion) |
| 17 | Climber (Kraken X60) |

**Motor Channels Used:** 17

---

#### Electronics

| Channel | Load |
|--------:|------|
| 18 | RoboRIO |
| 19 | Mini Power Module #1 |
| 20 | Mini Power Module #2 |

**Electronics Channels Used:** 3

---

### Channel Usage Summary

| Category | Channels |
|--------|----------|
| Motors | 17 |
| Electronics | 3 |
| **Total Used** | **20** |
| **Mini PDH Capacity** | **24** |
| **Remaining** | **3** |

---

### Missing / Unallocated Loads

The following required loads **do not fit** on a single Mini PDH:

- ❌ No spare channels for:
  - additional mechanisms
  - testing motors
  - future expansion
- ❌ No margin for:
  - temporary bring-up hardware
  - inspection-required changes
- ❌ Risky consolidation of critical systems onto minimal remaining channels

---


## Implementation Status

| Subsystem | Constants | Hardware IO | Status |
|-----------|-----------|-------------|--------|
| Swerve | TunerConstants.java | CommandSwerveDrivetrain | Complete |
| Intake | Complete | IntakeIOHardware.java | Partial |
| Indexer | Hopper | Needs update | IndexerIOHardware.java | Needs update |
| Shooter | Complete | ShooterIOHardware.java | Complete |
| Climber | Complete | ClimberIOHardware.java | Not implemented |
| Vision | Complete | VisionIOLimelight.java | Complete |

---

## Notes for Design & Controls

- Intake uses 2 motors total (1 rotator + 1 slide motor)
- Shooter flywheels are leader–follower and mechanically belted
- Hood uses external ThroughBore encoder for accurate position control 
- Hopper conveyor uses Minion + TalonFXS
- Swerve runs entirely on CANivore for performance isolation
