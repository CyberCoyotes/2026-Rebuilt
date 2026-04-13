# Hardware Reference  

A central location for motors, motor controllers, sensors, and CAN devices for quick reference.

---

## CAN ID Quick Reference — keep in sync with source constants ##

### candrive bus (CANivore) ###

| ID | Name | Type |
|----|------|------|
| 1 | FL Drive Motor | Kraken X60 (`TunerConstants.kFrontLeftDriveMotorId`) |
| 2 | FR Steer Motor | Kraken X60 (`TunerConstants.kFrontRightSteerMotorId`) |
| 3 | FL CANcoder | `TunerConstants.kFrontLeftEncoderId` |
| 4 | FR Drive Motor | Kraken X60 (`TunerConstants.kFrontRightDriveMotorId`) |
| 5 | FL Steer Motor | Kraken X60 (`TunerConstants.kFrontLeftSteerMotorId`) |
| 6 | FR CANcoder | `TunerConstants.kFrontRightEncoderId` |
| 7 | BL Drive Motor | Kraken X60 (`TunerConstants.kBackLeftDriveMotorId`) |
| 8 | BL Steer Motor | Kraken X60 (`TunerConstants.kBackLeftSteerMotorId`) |
| 9 | BL CANcoder | `TunerConstants.kBackLeftEncoderId` |
| 10 | BR Drive Motor | Kraken X60 (`TunerConstants.kBackRightDriveMotorId`) |
| 11 | BR Steer Motor | Kraken X60 (`TunerConstants.kBackRightSteerMotorId`) |
| 12 | BR CANcoder | `TunerConstants.kBackRightEncoderId` |
| 14 | Pigeon 2 IMU | `TunerConstants.kPigeonId` |
| 15 | CANdle LEDs | `Led.CANDLE_ID` |
  
### rio bus ### 

| ID | Name | Type |
|----|------|------|
| 20 | Intake Roller Left | Kraken X60 (`Intake.ROLLER_LEFT_MOTOR_ID`) |
<!-- | 21 | Intake Roller Right | Kraken X44 (`Intake.ROLLER_RIGHT_MOTOR_ID`) | -->
| 22 | Intake Slide | Kraken X44 (`Intake.SLIDE_MOTOR_ID`) |
| 23 | Kicker Left | Kraken X60 (`Indexer.KICKER_LEFT_MOTOR_ID`) |
| 24 | Kicker Right | Kraken X60 (`Indexer.KICKER_RIGHT_MOTOR_ID`) |
| 25 | Flywheel Left | Kraken X60 (`Shooter.FLYWHEEL_LEFT_MOTOR_ID`) |
| 26 | Flywheel Right | Kraken X60 (`Shooter.FLYWHEEL_RIGHT_MOTOR_ID`) |
| 27 | Conveyor | Kraken X44 (`Indexer.CONVEYOR_MOTOR_ID`) |
| 28 | Hood | Minion/TalonFXS (`Shooter.HOOD_MOTOR_ID`) |
| 42 | Chute ToF CANrange | `Indexer.CHUTE_TOF_ID` |


### Hood Details

- **Motor:** Minion  
- **Gearbox:** Cycloidal  
- **Reduction:** 23:1  
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
| Kraken X44 | TalonFX | 6 | Intake (2), Drive (4) |
| Falcon 500 | TalonFX | 3 | Shooter flywheels |
| Minion | TalonFXS | 2 | Shooter hood, Hopper conveyor |

**Total Motors:** 17

---

### Sensors by Type

| Sensor Type | Count | Subsystems |
|-------------|-------|------------|
| CANrange ToF | 1 | Indexer (1) |
| CANcoder | 4 | Swerve azimuth |
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
- Hopper conveyor uses Minion + TalonFXS
- Swerve runs entirely on CANivore for performance isolation
