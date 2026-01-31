package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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
    private final TalonFX hoodMotor;
    private final TalonFX counterWheelMotor;

    // ===== Control Requests =====
    // FOC (Field Oriented Control) for flywheels - smoother, more efficient
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
    private final VelocityVoltage counterWheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);

    // ===== Status Signals (for efficient reading) =====
    // Flywheel A
    private final StatusSignal<?> flywheelAVelocity;
    private final StatusSignal<?> flywheelAVoltage;
    private final StatusSignal<?> flywheelACurrent;
    private final StatusSignal<?> flywheelATemp;

    // Flywheel B
    private final StatusSignal<?> flywheelBVelocity;
    private final StatusSignal<?> flywheelBVoltage;
    private final StatusSignal<?> flywheelBCurrent;
    private final StatusSignal<?> flywheelBTemp;

    // Flywheel C
    private final StatusSignal<?> flywheelCVelocity;
    private final StatusSignal<?> flywheelCVoltage;
    private final StatusSignal<?> flywheelCCurrent;
    private final StatusSignal<?> flywheelCTemp;

    // Hood
    private final StatusSignal<?> hoodPosition;
    private final StatusSignal<?> hoodVoltage;
    private final StatusSignal<?> hoodCurrent;

    // Counter-wheel
    private final StatusSignal<?> counterWheelVelocity;
    private final StatusSignal<?> counterWheelVoltage;
    private final StatusSignal<?> counterWheelCurrent;

    // ===== Conversion Constants =====
    /** Gear ratio from motor to flywheel (motor rotations per flywheel rotation) */
    private static final double FLYWHEEL_GEAR_RATIO = 1.5;  // TODO: Measure actual ratio

    /** Gear ratio from motor to hood (motor rotations per degree of hood movement) */
    private static final double HOOD_GEAR_RATIO = 100.0;  // TODO: Measure actual ratio

    /**
     * Creates a new ShooterIOTalonFX instance.
     * Configures all motors to known good states.
     */
    public ShooterIOHardware() {
        // Create motor objects
        // flywheelMotorA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, Constants.kCANBus); // deprecated constructor
        flywheelMotorA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID); // TODO add new CANbus arg 
        flywheelMotorB = new TalonFX(Constants.Shooter.FLYWHEEL_B_MOTOR_ID); // TODO add new CANbus arg 
        flywheelMotorC = new TalonFX(Constants.Shooter.FLYWHEEL_C_MOTOR_ID); // TODO add new CANbus arg 
        hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);
        counterWheelMotor = new TalonFX(Constants.Shooter.COUNTER_WHEEL_MOTOR_ID);
        // https://v6.docs.ctr-electronics.com/en/stable/docs/yearly-changes/yearly-changelog.html#talon-fx-improvements

        // Apply configurations from centralized config class
        flywheelMotorA.getConfigurator().apply(TalonFXConfigs.shooterFlywheelConfig());
        flywheelMotorB.getConfigurator().apply(TalonFXConfigs.shooterFlywheelConfig());
        flywheelMotorC.getConfigurator().apply(TalonFXConfigs.shooterFlywheelConfig());
        hoodMotor.getConfigurator().apply(TalonFXConfigs.shooterHoodConfig());
        counterWheelMotor.getConfigurator().apply(TalonFXConfigs.shooterFlywheelConfig());

        // Set follower relationships (B and C follow A)
        // Aligned = same direction as leader (not opposed)
        flywheelMotorB.setControl(new Follower(
            Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));
        flywheelMotorC.setControl(new Follower(
            Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));

        // Get status signals for efficient reading
        // Flywheel A (leader)
        flywheelAVelocity = flywheelMotorA.getVelocity();
        flywheelAVoltage = flywheelMotorA.getMotorVoltage();
        flywheelACurrent = flywheelMotorA.getSupplyCurrent();
        flywheelATemp = flywheelMotorA.getDeviceTemp();

        // Flywheel B (follower)
        flywheelBVelocity = flywheelMotorB.getVelocity();
        flywheelBVoltage = flywheelMotorB.getMotorVoltage();
        flywheelBCurrent = flywheelMotorB.getSupplyCurrent();
        flywheelBTemp = flywheelMotorB.getDeviceTemp();

        // Flywheel C (follower)
        flywheelCVelocity = flywheelMotorC.getVelocity();
        flywheelCVoltage = flywheelMotorC.getMotorVoltage();
        flywheelCCurrent = flywheelMotorC.getSupplyCurrent();
        flywheelCTemp = flywheelMotorC.getDeviceTemp();

        // Hood
        hoodPosition = hoodMotor.getPosition();
        hoodVoltage = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getSupplyCurrent();

        // Counter-wheel
        counterWheelVelocity = counterWheelMotor.getVelocity();
        counterWheelVoltage = counterWheelMotor.getMotorVoltage();
        counterWheelCurrent = counterWheelMotor.getSupplyCurrent();

        // Configure update frequencies for better performance
        // Critical signals: 100Hz, Less critical: 50Hz, Temperature: 4Hz
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,  // 100Hz for velocity/position/voltage (critical for control)
            flywheelAVelocity, flywheelAVoltage,
            flywheelBVelocity, flywheelBVoltage,
            flywheelCVelocity, flywheelCVoltage,
            hoodPosition, hoodVoltage,
            counterWheelVelocity, counterWheelVoltage
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,  // 50Hz for current
            flywheelACurrent, flywheelBCurrent, flywheelCCurrent,
            hoodCurrent, counterWheelCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            4.0,  // 4Hz for temperature
            flywheelATemp, flywheelBTemp, flywheelCTemp
        );

        // Optimize bus utilization
        flywheelMotorA.optimizeBusUtilization();
        flywheelMotorB.optimizeBusUtilization();
        flywheelMotorC.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
        counterWheelMotor.optimizeBusUtilization();

        // Zero hood encoder at startup (assumes hood is at home position)
        hoodMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Refresh all status signals efficiently
        BaseStatusSignal.refreshAll(
            // Flywheel A
            flywheelAVelocity, flywheelAVoltage, flywheelACurrent, flywheelATemp,
            // Flywheel B
            flywheelBVelocity, flywheelBVoltage, flywheelBCurrent, flywheelBTemp,
            // Flywheel C
            flywheelCVelocity, flywheelCVoltage, flywheelCCurrent, flywheelCTemp,
            // Hood
            hoodPosition, hoodVoltage, hoodCurrent,
            // Counter-wheel
            counterWheelVelocity, counterWheelVoltage, counterWheelCurrent
        );

        // Convert motor velocities to flywheel RPM
        double flywheelARPM = rpsToRPM(flywheelAVelocity.getValueAsDouble()) / FLYWHEEL_GEAR_RATIO;
        double flywheelBRPM = rpsToRPM(flywheelBVelocity.getValueAsDouble()) / FLYWHEEL_GEAR_RATIO;
        double flywheelCRPM = rpsToRPM(flywheelCVelocity.getValueAsDouble()) / FLYWHEEL_GEAR_RATIO;

        // Individual flywheel data
        inputs.flywheelAVelocityRPM = flywheelARPM;
        inputs.flywheelBVelocityRPM = flywheelBRPM;
        inputs.flywheelCVelocityRPM = flywheelCRPM;

        // Average flywheel data
        inputs.flywheelVelocityRPM = (flywheelARPM + flywheelBRPM + flywheelCRPM) / 3.0;
        inputs.flywheelAppliedVolts = (flywheelAVoltage.getValueAsDouble() +
                                       flywheelBVoltage.getValueAsDouble() +
                                       flywheelCVoltage.getValueAsDouble()) / 3.0;
        inputs.flywheelCurrentAmps = (flywheelACurrent.getValueAsDouble() +
                                      flywheelBCurrent.getValueAsDouble() +
                                      flywheelCCurrent.getValueAsDouble());  // Sum, not average
        inputs.flywheelTempCelsius = Math.max(Math.max(
            flywheelATemp.getValueAsDouble(),
            flywheelBTemp.getValueAsDouble()),
            flywheelCTemp.getValueAsDouble());  // Highest temp

        // Hood data (convert motor rotations to degrees)
        inputs.hoodAngleDegrees = motorRotationsToDegrees(hoodPosition.getValueAsDouble());
        inputs.hoodAppliedVolts = hoodVoltage.getValueAsDouble();
        inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();

        // Counter-wheel data
        inputs.counterWheelVelocityRPM = rpsToRPM(counterWheelVelocity.getValueAsDouble());
        inputs.counterWheelAppliedVolts = counterWheelVoltage.getValueAsDouble();
        inputs.counterWheelCurrentAmps = counterWheelCurrent.getValueAsDouble();
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
    public void setHoodPose(double degrees) {
        // Convert degrees to motor rotations (accounting for gear ratio)
        double motorRotations = degreesToMotorRotations(degrees);

        // Set position
        hoodMotor.setControl(hoodPositionRequest.withPosition(motorRotations));
    }

    @Override
    public void setCounterWheelVelocity(double rpm) {
        // Convert RPM to RPS
        double rps = rpmToRPS(rpm);

        // Set velocity
        counterWheelMotor.setControl(counterWheelVelocityRequest.withVelocity(rps));
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

    /** Converts hood angle in degrees to motor rotations */
    private double degreesToMotorRotations(double degrees) {
        return degrees * HOOD_GEAR_RATIO / 360.0;
    }

    /** Converts motor rotations to hood angle in degrees */
    private double motorRotationsToDegrees(double rotations) {
        return rotations * 360.0 / HOOD_GEAR_RATIO;
    }
}
