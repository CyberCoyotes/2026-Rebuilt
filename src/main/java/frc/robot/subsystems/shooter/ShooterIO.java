package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * ShooterIO - Hardware abstraction interface for the shooter subsystem.
 *
 * The shooter launches game pieces using:
 * - Flywheel motors: 3x TalonFX (A=leader, B/C=followers) spin up to launch game pieces
 * - Hood motor: TalonFXS adjusts launch angle for different distances
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
        // ===== Flywheel Data (leader motor A only; B/C are followers) =====
        /** Leader motor velocity in RPM (motor shaft, no gear ratio applied) */
        public double flywheelLeaderMotorRPM = 0.0;

        /** Leader motor velocity in RPS (native TalonFX unit) */
        public double flywheelLeaderMotorRPS = 0.0;

        /** Leader motor velocity in RPS (alias for logging compatibility) */
        public double flywheelMotorRPS = 0.0;

        /** Flywheel applied voltage */
        public double flywheelAppliedVolts = 0.0;

        /** Flywheel supply current in amps */
        public double flywheelCurrentAmps = 0.0;

        // ===== Hood Data =====
        /** Hood position in raw motor rotations (0 = home, ~9.14 = max) */
        public double hoodPositionRotations = 0.0;

        /** Hood angle in degrees (approximate, derived from rotations) */
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
     * Sets the hood target pose in raw motor rotations.
     * Uses position closed-loop control.
     *
     * @param rawPosition Target pose in motor rotations (MIN_HOOD_POSE_ROT to MAX_HOOD_POSE_ROT)
     */
    default void setHoodPose(double rawPosition) {}

    /**
     * Sets the hood motor to a fixed voltage (open-loop).
     * Used for safe, slow movement when finding hood travel limits.
     *
     * @param volts Voltage to apply (positive = forward, negative = reverse)
     */
    default void setHoodVoltage(double volts) {}

    /**
     * Stops all shooter motors (flywheels, hood).
     */
    default void stop() {}
}
