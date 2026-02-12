package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Centralized TalonFX motor configurations for all subsystems.
 *
 * This class provides pre-configured TalonFXConfiguration objects for different
 * motor use cases across the robot. Centralizing configs here makes it easy to:
 * - Tune all similar motors at once
 * - Maintain consistency across subsystems
 * - See all motor settings in one place
 *
 * Pattern used by: FRC 254, 1678, 2910, and other top teams
 */
public class TalonFXConfigs {

    /**
     * Configuration for shooter flywheel motors.
     *
     * Features:
     * - Velocity control with FOC (Field Oriented Control)
     * - Coast mode (flywheels keep spinning when disabled)
     * - High current limits for rapid spinup
     * - Slot 0 PID for velocity control
     *
     * @return Configured TalonFXConfiguration for flywheel use
     */
    public static TalonFXConfiguration shooterFlywheelConfig() {
        var config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;  // Coast to keep spinning
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO : Verify direction on robot

        // Current limits - high for flywheels
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Stator (output) current limit
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Velocity PID - Slot 0 (tune these values!)
        config.Slot0.kP = 0.1;   // TODO: Tune on real robot
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;  // Feedforward velocity term
        config.Slot0.kS = 0.0;   // Feedforward static friction term

        return config;
    }

    /**
     * Configuration for indexer motors (floor and feeder).
     *
     * Features:
     * - Percent output control
     * - Brake mode (stop game pieces quickly)
     * - Moderate current limits
     *
     * @return Configured TalonFXConfiguration for indexer use
     */
    public static TalonFXConfiguration indexerConfig() {
        var config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Brake to stop game pieces
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits - moderate for indexer
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Stator current limit
        config.CurrentLimits.StatorCurrentLimit = 50.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Voltage compensation for consistent behavior
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }

    /**
     * Configuration for intake motors.
     *
     * Features:
     * - Percent output control
     * - Brake mode
     * - Lower current limits (intake doesn't need high power)
     *
     * @return Configured TalonFXConfiguration for intake use
     */
    public static TalonFXConfiguration intakeConfig() {
        var config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits - lower for intake
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 50.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        return config;
    }

    /**
     * Configuration for hood adjustment motor.
     *
     * Features:
     * - Position control with soft limits
     * - Brake mode (hold position)
     * - Lower current limits
     * - Slot 0 PID for position control
     *
     * @return Configured TalonFXConfiguration for hood use
     */
    public static TalonFXConfiguration shooterHoodConfig() {
        var config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Hold position
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: Verify direction on robot

        // Current limits - lower for hood
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Position PID - Slot 0 (tune these values!)
        config.Slot0.kP = 1.0;   // TODO: Tune on real robot
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        // Soft limits (TODO: set based on physical hood range)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;  // Enable after tuning
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100.0;  // TODO: Measure
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;  // Enable after tuning
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        return config;
    }

    /**
     * Configuration for climber motors.
     *
     * Features:
     * - Brake mode (safety - hold robot weight)
     * - High current limits
     * - Soft limits for safety
     *
     * @return Configured TalonFXConfiguration for climber use
     */
    public static TalonFXConfiguration climberConfig() {
        var config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // CRITICAL: Brake for safety
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits - high for climber
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Soft limits (TODO: set based on physical climber range)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;  // Enable after tuning
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100.0;  // TODO: Measure
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;  // Enable after tuning
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        return config;
    }
}