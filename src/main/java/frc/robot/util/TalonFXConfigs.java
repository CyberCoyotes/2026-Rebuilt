package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
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
    // public static TalonFXConfiguration flywheelConfig() {
    //     var config = new TalonFXConfiguration();

    //     // Motor output configuration
    //     config.MotorOutput.NeutralMode = NeutralModeValue.Coast;  // Coast to keep spinning
    //     config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO : Verify direction on robot

    //     // Current limits - high for flywheels
    //     config.CurrentLimits.SupplyCurrentLimit = 20.0; // TODO tune
    //     config.CurrentLimits.SupplyCurrentLimitEnable = true;

    //     // Stator (output) current limit
    //     config.CurrentLimits.StatorCurrentLimit = 20.0; // TODO Tune
    //     config.CurrentLimits.StatorCurrentLimitEnable = true;

    //     // Velocity PID - Slot 0 (tune these values!)
    //     config.Slot0.kP = 0.1;   // TODO: Tune on real robot
    //     config.Slot0.kI = 0.0;
    //     config.Slot0.kD = 0.0;
    //     config.Slot0.kV = 0.12;  // Feedforward velocity term
    //     config.Slot0.kS = 0.0;   // Feedforward static friction term

    //     // Closed-loop ramp rate — limits how fast the PID output voltage can change.
    //     // Prevents belt slipping from sudden torque spikes during flywheel acceleration.
    //     // Shorten this value once belt mechanics are improved.
    //     config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 3.0;  // TODO seconds from 0 to full voltage

    //     return config;
    // }

    /**
     * Configuration for indexer motor (TalonFX - feeds pieces to shooter).
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
     * Configuration for conveyor motor (TalonFXS with Minion motor).
     *
     * Features:
     * - TalonFXS controller with Minion_JST motor arrangement
     * - Percent output control
     * - Brake mode (stop game pieces quickly)
     * - Moderate current limits
     *
     * @return Configured TalonFXSConfiguration for conveyor use
     */
    public static TalonFXSConfiguration conveyorConfig() {
        var config = new TalonFXSConfiguration();

        // TalonFXS motor arrangement - Minion connected via JST
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Brake to stop game pieces
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limits - moderate for conveyor
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Stator current limit
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Voltage compensation for consistent behavior
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }

    /**
     * Configuration for intake roller motors.
     *
     * Features:
     * - Percent output control
     * - Brake mode
     * - Lower current limits (intake doesn't need high power)
     *
     * @return Configured TalonFXConfiguration for intake roller use
     */
    public static TalonFXConfiguration rollerConfig() {
        var config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current limits - lower for intake
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        return config;
    }

      /**
     * Configuration for intake slide motors.
     *
     * Features:
     * - Percent output control
     * - Brake mode
     * - Lower current limits (intake slide doesn't need high power)
     *
     * @return Configured TalonFXConfiguration for intake slide use
     */
    public static TalonFXConfiguration slideConfig() {
        var config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limits - lower for intake
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.90;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.00;

        config.Slot0.kP = 2.0;   // TODO: Tune — increase if response is too sluggish
        config.Slot0.kI = 0.0;   // TODO: Tune — helps eliminate steady-state error from friction/gravity
        config.Slot0.kD = 0.0;

        
        // Set MotionMagicVoltage settings
        config.MotionMagic.MotionMagicCruiseVelocity = 16; // TODO tune the slide speed - 8 is a good starting point for smooth movement without slipping
        config.MotionMagic.MotionMagicAcceleration = 16; // TODO tune the slide acceleration - 4 is a good starting point for smooth movement without slipping
        config.MotionMagic.MotionMagicJerk = 0;

        return config;
    }

    /**
     * Configuration for hood adjustment motor (TalonFXS with Minion motor).
     *
     * Features:
     * - TalonFXS controller with Minion_JST motor arrangement
     * - Position control with soft limits
     * - Brake mode (hold position)
     * - Lower current limits
     * - Slot 0 PID for position control
     *
     * @return Configured TalonFXSConfiguration for hood use
     */
    public static TalonFXSConfiguration hoodConfig() {
        var config = new TalonFXSConfiguration();

        // TalonFXS motor arrangement - Minion connected via JST
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Hold position
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: Verify direction on robot

        // Voltage limits - cap output for safe hood movement during testing
        config.Voltage.PeakForwardVoltage = 4.0;   // TODO: Increase after hood travel is verified. 4 has been safe for testing.
        config.Voltage.PeakReverseVoltage = -4.0;

        // Current limits - lower for hood
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Position PID - Slot 0
        config.Slot0.kP = 1.0;   // TODO: Tune — increase if response is too sluggish
        config.Slot0.kI = 0.75;   // TODO: Tune — helps eliminate steady-state error from friction/gravity
        config.Slot0.kD = 0.0;

        // Soft limits (TODO: set based on physical hood range)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;  // Enable after tuning
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9.15;  // Raw units
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;  // Enable after tuning
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