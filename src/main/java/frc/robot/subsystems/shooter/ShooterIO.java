package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * ShooterIO - Hardware abstraction interface for the shooter subsystem.
 *
 * The shooter launches game pieces using:
 * - Flywheel motors: Spin up to launch game pieces at variable velocities
 * - Hood motor: Adjusts launch angle for different distances
 * - Counter-wheel motor: Provides backspin/control (optional)
 *
 * PATTERN: IO Interface
 * - ShooterIO: Interface defining what shooter hardware can do
 * - ShooterIOHardware: Real hardware implementation with CTRE motors
 * - ShooterIOSim: Simulation for testing without hardware
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */
public interface ShooterIO {

    /**
     * ShooterIOInputs - Container for all shooter sensor data.
     *
     * This class holds all data we read from the shooter hardware each cycle.
     * Implements LoggableInputs for automatic AdvantageKit logging and replay.
     */
    @AutoLog
    class ShooterIOInputs {
        // ===== Main Flywheel Data (average of 3 motors) =====
        /** Average flywheel velocity in RPM */
        public double flywheelVelocityRPM = 0.0;

        /** Average flywheel applied voltage */
        public double flywheelAppliedVolts = 0.0;

        /** Average flywheel supply current in amps */
        public double flywheelCurrentAmps = 0.0;

        /** Average flywheel temperature in Celsius */
        public double flywheelTempCelsius = 0.0;

        /** Average raw motor velocity in RPS (native TalonFX unit, before gear ratio conversion) */
        public double flywheelMotorRPS = 0.0;

        // ===== Individual Flywheel Data (for diagnostics) =====
        /** Flywheel A velocity in RPM */
        public double flywheelAVelocityRPM = 0.0;

        /** Flywheel B velocity in RPM */
        public double flywheelBVelocityRPM = 0.0;

        /** Flywheel C velocity in RPM */
        public double flywheelCVelocityRPM = 0.0;

        // ===== Hood Data =====
        /** Hood angle in degrees from motor encoder (0 = home position) */
        public double hoodAngleDegrees = 0.0;

        /** Hood applied voltage */
        public double hoodAppliedVolts = 0.0;

        /** Hood supply current in amps */
        public double hoodCurrentAmps = 0.0;

        // ===== WCP ThroughBore Encoder (secondary feedback via CANcoder) =====
        /** Hood absolute position from ThroughBore encoder in rotations (0.0 to 1.0) */
        public double hoodThroughBorePositionRotations = 0.0;

        /** Hood absolute position from ThroughBore encoder in degrees */
        public double hoodThroughBorePositionDegrees = 0.0;

        /** Whether the ThroughBore encoder (CANcoder) is connected */
        public boolean hoodThroughBoreConnected = false;

        // ===== Counter-Wheel Data (optional) =====
        /** Counter-wheel velocity in RPM */
        public double counterWheelVelocityRPM = 0.0;

        /** Counter-wheel applied voltage */
        public double counterWheelAppliedVolts = 0.0;

        /** Counter-wheel supply current in amps */
        public double counterWheelCurrentAmps = 0.0;
    }

    /**
     * Updates inputs from hardware.
     * Called periodically (every 20ms) by ShooterSubsystem.
     *
     * @param inputs The ShooterIOInputs object to populate with current data
     */
    default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Sets the flywheel target velocity in RPM.
     * Uses velocity closed-loop control with FOC for smooth acceleration.
     *
     * @param rpm Target velocity in rotations per minute (positive = forward, negative = reverse)
     */
    default void setFlywheelVelocity(double rpm) {}

    /**
     * Stops the flywheel motors (velocity = 0).
     */
    default void stopFlywheels() {}

    /**
     * Sets the hood target pose in degrees.
     * Uses position closed-loop control.
     *
     * @param degrees Target pose in degrees (MIN_POSE to MAX_POSE range)
     */
    default void setHoodPose(double degrees) {}

    /**
     * Sets the counter-wheel velocity in RPM (optional).
     *
     * @param rpm Target velocity in rotations per minute
     */
    default void setCounterWheelVelocity(double rpm) {}

    /**
     * Sets the hood motor to a fixed voltage (open-loop).
     * Used for safe, slow movement when finding hood travel limits.
     *
     * @param volts Voltage to apply (positive = forward, negative = reverse)
     */
    default void setHoodVoltage(double volts) {}

    /**
     * Stops all shooter motors (flywheels, hood, counter-wheel).
     */
    default void stop() {}
}
