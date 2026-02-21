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
    public static TalonFXConfiguration flywheelConfig() {
        var config = new TalonFXConfiguration();

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: Verify direction on robot

        // Current limits - high for flywheels
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Velocity PID - Slot 0
        config.Slot0.kP = 0.1;   // TODO: Tune on real robot
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;  // Feedforward velocity term
        config.Slot0.kS = 0.0;   // Feedforward static friction term

        // Closed-loop ramp rate — limits how fast PID output voltage can change.
        // Prevents belt slipping from sudden torque spikes during flywheel acceleration.
        // Currently 2.0s for testing. Reduce toward 0.25-0.5s once belts are trusted.
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 2.0; // TODO: Reduce once belts are trusted

        return config;
    }

    /**
     * Configuration for indexer motor (TalonFX - feeds pieces to shooter).
     *
     * @return Configured TalonFXConfiguration for indexer use
     */
    public static TalonFXConfiguration indexerConfig() {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 50.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }

    /**
     * Configuration for conveyor motor (TalonFXS with Minion motor).
     *
     * @return Configured TalonFXSConfiguration for conveyor use
     */
    public static TalonFXSConfiguration conveyorConfig() {
        var config = new TalonFXSConfiguration();

        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }

    /**
     * Configuration for intake roller motors.
     *
     * @return Configured TalonFXConfiguration for intake roller use
     */
    public static TalonFXConfiguration rollerConfig() {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        return config;
    }

    /**
     * Configuration for intake slide motors.
     *
     * @return Configured TalonFXConfiguration for intake slide use
     */
    public static TalonFXConfiguration slideConfig() {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.-.1;

        config.Slot0.kP = 3.0;   // TODO: Tune
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 16; // TODO: Tune
        config.MotionMagic.MotionMagicAcceleration = 16;   // TODO: Tune
        config.MotionMagic.MotionMagicJerk = 0;

        return config;
    }

    /**
     * Configuration for hood adjustment motor (TalonFXS with Minion motor).
     *
     * NOTE: Soft limits are currently DISABLED for initial testing.
     * Once hood travel is verified and zeroing is confirmed correct,
     * re-enable and set thresholds:
     *   config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
     *   config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9.15;
     *   config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
     *   config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
     *
     * NOTE: kI is set to 0.0. The previous value of 0.75 caused the integrator
     * to wind up rapidly and slam the hood to its limit. Re-introduce kI
     * cautiously (start at 0.01 or lower) only if steady-state error persists
     * after kP tuning.
     *
     * @return Configured TalonFXSConfiguration for hood use
     */
    public static TalonFXSConfiguration hoodConfig() {
        var config = new TalonFXSConfiguration();

        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: Verify direction on robot

        // Voltage limits — capped for safe hood movement during testing
        config.Voltage.PeakForwardVoltage = 4.0;   // TODO: Increase after hood travel is verified
        config.Voltage.PeakReverseVoltage = -4.0;

        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Position PID - Slot 0
        config.Slot0.kP = 1.0;   // TODO: Tune — increase if response is too sluggish
        config.Slot0.kI = 0.0;   // WARNING: Do not increase without caution — high kI winds up fast
        config.Slot0.kD = 0.0;

        // Soft limits — DISABLED for initial testing (see notes above)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        return config;
    }

    /**
     * Configuration for climber motors.
     *
     * @return Configured TalonFXConfiguration for climber use
     */
    public static TalonFXConfiguration climberConfig() {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // CRITICAL: Brake for safety
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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