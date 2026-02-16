package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.util.TalonFXConfigs;

/**
 * ShooterIOHardware - Real hardware implementation using CTRE TalonFX motors.
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Velocity control with FOC for smooth, powerful flywheel acceleration
 * - Position control for precise hood angle adjustment
 * - Optimized status signal updates for performance
 * - All telemetry logged via AdvantageKit
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */
public class ShooterIOHardware implements ShooterIO {

    // ===== Hardware =====
    private final TalonFX flywheelMotorA;
    private final TalonFX flywheelMotorB;
    private final TalonFX flywheelMotorC;
    private final TalonFXS hoodMotor;
    private final TalonFX counterWheelMotor;
    private final CANcoder hoodEncoder; // WCP ThroughBore Encoder via CANcoder

    // ===== Control Requests =====
    // FOC (Field Oriented Control) for flywheels - smoother, more efficient
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
    // private final VelocityVoltage counterWheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);

    // ===== Status Signals (for efficient reading) =====
    // Flywheel A
    private final StatusSignal<?> flywheelAVelocity;
    // private final StatusSignal<?> flywheelAVoltage;
    // private final StatusSignal<?> flywheelACurrent;
    // private final StatusSignal<?> flywheelATemp;

    // Flywheel B
    private final StatusSignal<?> flywheelBVelocity;
    // private final StatusSignal<?> flywheelBVoltage;
    // private final StatusSignal<?> flywheelBCurrent;
    // private final StatusSignal<?> flywheelBTemp;

    // Flywheel C
    private final StatusSignal<?> flywheelCVelocity;
    // private final StatusSignal<?> flywheelCVoltage;
    // private final StatusSignal<?> flywheelCCurrent;
    // private final StatusSignal<?> flywheelCTemp;

    // Hood
    private final StatusSignal<?> hoodPosition;
    private final StatusSignal<?> hoodVoltage;
    private final StatusSignal<?> hoodCurrent;

    // // Counter-wheel
    // private final StatusSignal<?> counterWheelVelocity;
    // private final StatusSignal<?> counterWheelVoltage;
    // private final StatusSignal<?> counterWheelCurrent;

    // WCP ThroughBore Encoder (CANcoder)
    private final StatusSignal<?> hoodEncoderAbsPosition;

    // ===== Conversion Constants =====
    /** Gear ratio from motor to flywheel (motor rotations per flywheel rotation) */
    private static final double FLYWHEEL_GEAR_RATIO = 1.5;  // TODO: Measure actual ratio

    // Hood gear ratio — NOT currently used. Hood values are raw motor rotations from Phoenix Tuner.
    // Uncomment if a mechanical gear ratio conversion is needed later.
    // private static final double HOOD_GEAR_RATIO = 100.0;  // TODO: Measure actual ratio 23:1 or 1:23

    /** 
     * Creates a new ShooterIOTalonFX instance.
     * Configures all motors to known good states.
     */
    public ShooterIOHardware() {
        // Create motor objects
        // flywheelMotorA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, Constants.kCANBus); // deprecated constructor
        flywheelMotorA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, Constants.CAN_BUS_NAME); // TODO add new CANbus arg 
        flywheelMotorB = new TalonFX(Constants.Shooter.FLYWHEEL_B_MOTOR_ID, Constants.CAN_BUS_NAME); // TODO add new CANbus arg 
        flywheelMotorC = new TalonFX(Constants.Shooter.FLYWHEEL_C_MOTOR_ID, Constants.CAN_BUS_NAME); // TODO add new CANbus arg 
        hoodMotor = new TalonFXS(Constants.Shooter.HOOD_MOTOR_ID, Constants.CAN_BUS_NAME);
        counterWheelMotor = new TalonFX(Constants.Shooter.COUNTER_WHEEL_MOTOR_ID, Constants.CAN_BUS_NAME);
        hoodEncoder = new CANcoder(Constants.Shooter.HOOD_POSE_ENCODER_ID, Constants.CAN_BUS_NAME);
        // https://v6.docs.ctr-electronics.com/en/stable/docs/yearly-changes/yearly-changelog.html#talon-fx-improvements

        // Configure WCP ThroughBore encoder (CANcoder)
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0; // Unsigned [0, 1) range
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // TODO: Verify direction on robot
        encoderConfig.MagnetSensor.MagnetOffset = 0.0; // TODO: Measure offset when hood is at home position
        hoodEncoder.getConfigurator().apply(encoderConfig);

        // Apply configurations from centralized config class
        flywheelMotorA.getConfigurator().apply(TalonFXConfigs.flywheelConfig());
        flywheelMotorB.getConfigurator().apply(TalonFXConfigs.flywheelConfig());
        flywheelMotorC.getConfigurator().apply(TalonFXConfigs.flywheelConfig());
        hoodMotor.getConfigurator().apply(TalonFXConfigs.hoodConfig());
        counterWheelMotor.getConfigurator().apply(TalonFXConfigs.flywheelConfig());

        // Set follower relationships (B and C follow A)
        // Aligned = same direction as leader (not opposed)
        flywheelMotorB.setControl(new Follower(
            Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned)); // 
        flywheelMotorC.setControl(new Follower(
            Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));

        // Get status signals for efficient reading
        // Flywheel A (leader)
        flywheelAVelocity = flywheelMotorA.getVelocity();
        // flywheelAVoltage = flywheelMotorA.getMotorVoltage();
        // flywheelACurrent = flywheelMotorA.getSupplyCurrent();
        // flywheelATemp = flywheelMotorA.getDeviceTemp();

        // Flywheel B (follower)
        flywheelBVelocity = flywheelMotorB.getVelocity();
        // flywheelBVoltage = flywheelMotorB.getMotorVoltage();
        // flywheelBCurrent = flywheelMotorB.getSupplyCurrent();
        // flywheelBTemp = flywheelMotorB.getDeviceTemp();

        // Flywheel C (follower)
        flywheelCVelocity = flywheelMotorC.getVelocity();
        // flywheelCVoltage = flywheelMotorC.getMotorVoltage();
        // flywheelCCurrent = flywheelMotorC.getSupplyCurrent();
        // flywheelCTemp = flywheelMotorC.getDeviceTemp();

        // Hood
        hoodPosition = hoodMotor.getPosition();
        hoodVoltage = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getSupplyCurrent();

        // Counter-wheel
        // counterWheelVelocity = counterWheelMotor.getVelocity();
        // counterWheelVoltage = counterWheelMotor.getMotorVoltage();
        // counterWheelCurrent = counterWheelMotor.getSupplyCurrent();

        // WCP ThroughBore Encoder (CANcoder)
        hoodEncoderAbsPosition = hoodEncoder.getAbsolutePosition();

        // Configure update frequencies for better performance
        // Critical signals: 100Hz, Less critical: 50Hz, Temperature: 4Hz
        // BaseStatusSignal.setUpdateFrequencyForAll(
        //     100.0,  // 100Hz for velocity/position/voltage (critical for control)
        //     flywheelAVelocity,
        //     flywheelBVelocity,
        //     flywheelCVelocity,
        //     hoodPosition, hoodVoltage,
        //     hoodEncoderAbsPosition
        // );

        // BaseStatusSignal.setUpdateFrequencyForAll(
        //     50.0,  // 50Hz for current
        //     flywheelACurrent, flywheelBCurrent, flywheelCCurrent,
        //     hoodCurrent, counterWheelCurrent
        // );

        // BaseStatusSignal.setUpdateFrequencyForAll(
        //     4.0,  // 4Hz for temperature
        //     flywheelATemp, flywheelBTemp, flywheelCTemp
        // );

        // Optimize bus utilization
        flywheelMotorA.optimizeBusUtilization();
        flywheelMotorB.optimizeBusUtilization();
        flywheelMotorC.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
        counterWheelMotor.optimizeBusUtilization();
        hoodEncoder.optimizeBusUtilization();

        // Zero hood encoder at startup (assumes hood is at home position)
        hoodMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Refresh all status signals efficiently
        BaseStatusSignal.refreshAll(
            // Flywheel A
            flywheelAVelocity,
            // Flywheel B
            // flywheelBVelocity,
            // Flywheel C
            // flywheelCVelocity,
            // Hood
            hoodPosition, hoodVoltage, hoodCurrent,
            // Counter-wheel
            // WCP ThroughBore Encoder
            hoodEncoderAbsPosition
        );

        // Raw motor RPS (native TalonFX unit — no conversion)
        double motorARPS = flywheelAVelocity.getValueAsDouble();
        // double motorBRPS = flywheelBVelocity.getValueAsDouble();
        // double motorCRPS = flywheelCVelocity.getValueAsDouble();
        // inputs.flywheelMotorRPS = (motorARPS + motorBRPS + motorCRPS) / 3.0;

        // Convert motor velocities to flywheel RPM
        double flywheelARPM = rpsToRPM(motorARPS) / FLYWHEEL_GEAR_RATIO;
        // double flywheelBRPM = rpsToRPM(motorBRPS) / FLYWHEEL_GEAR_RATIO;
        // double flywheelCRPM = rpsToRPM(motorCRPS) / FLYWHEEL_GEAR_RATIO;

        // Individual flywheel data
        // inputs.flywheelAVelocityRPM = flywheelARPM;
        // inputs.flywheelBVelocityRPM = flywheelBRPM;
        // inputs.flywheelCVelocityRPM = flywheelCRPM;

        // Average flywheel data
        // inputs.flywheelVelocityRPM = (flywheelARPM + flywheelBRPM + flywheelCRPM) / 3.0;
        // inputs.flywheelAppliedVolts = (flywheelAVoltage.getValueAsDouble() +
        //                                flywheelBVoltage.getValueAsDouble() +
        //                                flywheelCVoltage.getValueAsDouble()) / 3.0;
        // inputs.flywheelCurrentAmps = (flywheelACurrent.getValueAsDouble() +
        //                               flywheelBCurrent.getValueAsDouble() +
        //                               flywheelCCurrent.getValueAsDouble());  // Sum, not average
        // inputs.flywheelTempCelsius = Math.max(Math.max(
        //     flywheelATemp.getValueAsDouble(),
        //     flywheelBTemp.getValueAsDouble()),
        //     flywheelCTemp.getValueAsDouble());  // Highest temp

        // Hood data — raw motor rotations, no conversion needed (matches Phoenix Tuner units)
        // inputs.hoodAngleDegrees = hoodPosition.getValueAsDouble();
        // inputs.hoodAppliedVolts = hoodVoltage.getValueAsDouble();
        // inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();

        // Counter-wheel data
        // inputs.counterWheelVelocityRPM = rpsToRPM(counterWheelVelocity.getValueAsDouble());
        // inputs.counterWheelAppliedVolts = counterWheelVoltage.getValueAsDouble();
        // inputs.counterWheelCurrentAmps = counterWheelCurrent.getValueAsDouble();

        // WCP ThroughBore Encoder data (secondary feedback)
        inputs.hoodThroughBorePositionRotations = hoodEncoderAbsPosition.getValueAsDouble();
        inputs.hoodThroughBorePositionDegrees = inputs.hoodThroughBorePositionRotations * 360.0;
        inputs.hoodThroughBoreConnected =
            hoodEncoderAbsPosition.getStatus() == StatusCode.OK;
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        // Convert RPM to motor RPS (accounting for gear ratio)
        double motorRPS = rpmToRPS(rpm) * FLYWHEEL_GEAR_RATIO;

        // Set velocity for leader motor (followers will match)
        flywheelMotorA.setControl(flywheelVelocityRequest.withVelocity(motorRPS));
    }

    @Override
    public void stopFlywheels() {
        // Stop flywheel motors (followers stop automatically)
        flywheelMotorA.stopMotor();
    }

    @Override
    public void setHoodPose(double rawPosition) {
        // rawPosition is in motor rotations as measured in Phoenix Tuner (0 to ~9.14)
        // No gear ratio conversion needed — values are already in motor units
        hoodMotor.setControl(hoodPositionRequest.withPosition(rawPosition));
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodMotor.setControl(hoodVoltageRequest.withOutput(volts));
    }

    @Override
    public void setCounterWheelVelocity(double rpm) {
        // Convert RPM to RPS
        double rps = rpmToRPS(rpm);
    }

    @Override
    public void stop() {
        flywheelMotorA.stopMotor();  // Followers stop automatically
        hoodMotor.stopMotor();
        counterWheelMotor.stopMotor();
    }

    // ===== Helper Methods for Unit Conversion =====

    /** Converts RPS (rotations per second) to RPM (rotations per minute) */
    private double rpsToRPM(double rps) {
        return rps * 60.0;
    }

    /** Converts RPM (rotations per minute) to RPS (rotations per second) */
    private double rpmToRPS(double rpm) {
        return rpm / 60.0;
    }

    // Hood gear ratio conversion helpers — NOT currently used.
    // Hood pose values (0 to ~9.14) are raw motor rotations from Phoenix Tuner,
    // so no conversion is needed. These are kept in case a mechanical gear ratio
    // is added later that requires converting between degrees and motor rotations.
    //
    // /** Converts hood angle in degrees to motor rotations */
    // private double degreesToMotorRotations(double degrees) {
    //     return degrees * HOOD_GEAR_RATIO / 360.0;
    // }
    //
    // /** Converts motor rotations to hood angle in degrees */
    // private double motorRotationsToDegrees(double rotations) {
    //     return rotations * 360.0 / HOOD_GEAR_RATIO;
    // }
}
