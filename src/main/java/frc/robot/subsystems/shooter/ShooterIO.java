package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * ShooterIO - Hardware abstraction interface for the shooter subsystem.
 *
 * The shooter launches game pieces using:
 * - Flywheel motors: 2x TalonFX (leader + 1 follower, opposite sides) spin up to launch game pieces
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
     *
     * PERFORMANCE NOTE:
     * Fields are split into "fast" (updated every cycle) and "slow" (updated at 10Hz).
     * The fast fields are control-critical; the slow fields are for diagnostics/dashboard.
     */
    @AutoLog
    class ShooterIOInputs implements LoggableInputs{
        // ===== Fast Fields (updated every 20ms cycle) =====
        // These drive closed-loop control and must be fresh every cycle.

        /** Leader motor velocity in RPM (motor shaft, no gear ratio applied) */
        public double flywheelLeaderMotorRPM = 0.0;

        /** Leader motor velocity in RPS (native TalonFX unit) */
        public double flywheelLeaderMotorRPS = 0.0;

        /** Flywheel applied voltage */
        public double flywheelAppliedVolts = 0.0;

        /** Hood position in raw motor rotations (0 = home, ~9.14 = max) */
        public double hoodPositionRotations = 0.0;

        // ===== Slow Fields (updated at 10Hz — diagnostics only) =====
        // These are for dashboard display, overcurrent detection, and logging.
        // Updated via updateSlowInputs() every 5th cycle.

        /** Flywheel supply current in amps */
        public double flywheelCurrentAmps = 0.0;

        /** Max flywheel motor temperature in Celsius across motors A/B/C */
        public double flywheelMaxTempCelsius = 0.0;

        /** Hood applied voltage */
        public double hoodAppliedVolts = 0.0;

        /** Hood supply current in amps */
        public double hoodCurrentAmps = 0.0;

        /** Hood angle in degrees (approximate, derived from rotations) */
        public double hoodAngleDegrees = 0.0;

    @Override
public void toLog(LogTable table) {
    // Fast Fields
    table.put("FlywheelLeaderMotorRPM", flywheelLeaderMotorRPM);
    table.put("FlywheelLeaderMotorRPS", flywheelLeaderMotorRPS);
    table.put("FlywheelAppliedVolts", flywheelAppliedVolts);
    table.put("HoodPositionRotations", hoodPositionRotations);

    // Slow Fields
    table.put("FlywheelCurrentAmps", flywheelCurrentAmps);
    table.put("FlywheelMaxTempCelsius", flywheelMaxTempCelsius);
    table.put("HoodAppliedVolts", hoodAppliedVolts);
    table.put("HoodCurrentAmps", hoodCurrentAmps);
    table.put("HoodAngleDegrees", hoodAngleDegrees);
}

@Override
public void fromLog(LogTable table) {
    // Fast Fields
    flywheelLeaderMotorRPM = table.get("FlywheelLeaderMotorRPM", flywheelLeaderMotorRPM);
    flywheelLeaderMotorRPS = table.get("FlywheelLeaderMotorRPS", flywheelLeaderMotorRPS);
    flywheelAppliedVolts = table.get("FlywheelAppliedVolts", flywheelAppliedVolts);
    hoodPositionRotations = table.get("HoodPositionRotations", hoodPositionRotations);

    // Slow Fields
    flywheelCurrentAmps = table.get("FlywheelCurrentAmps", flywheelCurrentAmps);
    flywheelMaxTempCelsius = table.get("FlywheelMaxTempCelsius", flywheelMaxTempCelsius);
    hoodAppliedVolts = table.get("HoodAppliedVolts", hoodAppliedVolts);
    hoodCurrentAmps = table.get("HoodCurrentAmps", hoodCurrentAmps);
    hoodAngleDegrees = table.get("HoodAngleDegrees", hoodAngleDegrees);
}
    }

    /**
     * Updates control-critical inputs from hardware.
     * Called every cycle (every 20ms) by ShooterSubsystem.
     * Only refreshes fast signals: flywheel velocity, flywheel voltage, hood position.
     *
     * @param inputs The ShooterIOInputs object to populate with current data
     */
    default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Updates diagnostic inputs from hardware.
     * Called at 10Hz (every 5th cycle) by ShooterSubsystem.
     * WHY SEPARATE? These signals are set to 10Hz update rate on the CAN bus,
     * so refreshing them every 20ms just reads stale cached values and wastes
     * JNI calls. Matching our read rate to the CAN update rate is cleaner.
     *
     * @param inputs The ShooterIOInputs object to populate with diagnostic data
     */
    default void updateSlowInputs(ShooterIOInputs inputs) {}

    /**
     * Sets the flywheel target velocity in RPM.
     * Uses the configured flywheel velocity closed-loop mode.
     *
     * @param rpm Target velocity in rotations per minute (positive = forward, negative = reverse)
     */
    default void setFlywheelVelocity(double rpm) {}

    /**
     * Sets the flywheel target velocity using VelocityTorqueCurrentFOC.
     *
     * NOTE: TorqueCurrentFOC requires CAN FD (CANivore bus).
     * On RIO CAN it will not perform as intended — for comparison testing only
     * until flywheel motors are moved to CANivore.
     *
     * kP units: Amps per RPS (NOT Volts/RPS — retune separately from VelocityVoltage gains).
     *
     * @param rpm Target velocity in rotations per minute
     */
    default void setFlywheelVelocityTorqueFOC(double rpm) {}
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
