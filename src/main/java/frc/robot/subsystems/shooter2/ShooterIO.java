package frc.robot.subsystems.shooter2;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * ShooterIO - Hardware abstraction interface for the shooter subsystem.
 *
 * The shooter launches game pieces using:
 * - 3x Flywheel motors: Spin up to launch game pieces at variable velocities
 * - 1x Hood motor: Adjusts launch angle for different distances
 * - 1x Counter-wheel motor: Provides backspin/control (optional)
 *
 * HARDWARE:
 * - 3x Falcon 500 / TalonFX (main flywheels) - velocity control with FOC
 * - 1x Minion motor (hood adjustment) - position control
 * - 1x Kraken X60 (counter-wheel) - velocity control
 *
 * PATTERN: IO Interface
 * - ShooterIO: Interface defining what shooter hardware can do
 * - ShooterIOTalonFX: Real hardware implementation with CTRE motors
 * - ShooterIOSim: Simulation for testing without hardware
 *
 * AUTHOR: @Isaak3
 */
public interface ShooterIO {

    /**
     * ShooterIOInputs - Container for all shooter sensor data.
     *
     * This class holds all data we read from the shooter hardware each cycle.
     * Implements LoggableInputs for automatic AdvantageKit logging and replay.
     */
    class ShooterIOInputs implements LoggableInputs {
        // ===== Main Flywheel Data (average of 3 motors) =====
        /** Average flywheel velocity in RPM */
        public double flywheelVelocityRPM = 0.0;

        /** Average flywheel applied voltage */
        public double flywheelAppliedVolts = 0.0;

        /** Average flywheel supply current in amps */
        public double flywheelCurrentAmps = 0.0;

        /** Average flywheel temperature in Celsius */
        public double flywheelTempCelsius = 0.0;

        // ===== Individual Flywheel Data (for diagnostics) =====
        /** Flywheel A velocity in RPM */
        public double flywheelAVelocityRPM = 0.0;

        /** Flywheel B velocity in RPM */
        public double flywheelBVelocityRPM = 0.0;

        /** Flywheel C velocity in RPM */
        public double flywheelCVelocityRPM = 0.0;

        // ===== Hood Data =====
        /** Hood angle in degrees (0 = home position) */
        public double hoodAngleDegrees = 0.0;

        /** Hood applied voltage */
        public double hoodAppliedVolts = 0.0;

        /** Hood supply current in amps */
        public double hoodCurrentAmps = 0.0;

        // ===== Counter-Wheel Data (optional) =====
        /** Counter-wheel velocity in RPM */
        public double counterWheelVelocityRPM = 0.0;

        /** Counter-wheel applied voltage */
        public double counterWheelAppliedVolts = 0.0;

        /** Counter-wheel supply current in amps */
        public double counterWheelCurrentAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            // Main flywheel (averaged)
            table.put("FlywheelVelocityRPM", flywheelVelocityRPM);
            table.put("FlywheelAppliedVolts", flywheelAppliedVolts);
            table.put("FlywheelCurrentAmps", flywheelCurrentAmps);
            table.put("FlywheelTempCelsius", flywheelTempCelsius);

            // Individual flywheels (for diagnostics)
            table.put("FlywheelAVelocityRPM", flywheelAVelocityRPM);
            table.put("FlywheelBVelocityRPM", flywheelBVelocityRPM);
            table.put("FlywheelCVelocityRPM", flywheelCVelocityRPM);

            // Hood
            table.put("HoodAngleDegrees", hoodAngleDegrees);
            table.put("HoodAppliedVolts", hoodAppliedVolts);
            table.put("HoodCurrentAmps", hoodCurrentAmps);

            // Counter-wheel
            table.put("CounterWheelVelocityRPM", counterWheelVelocityRPM);
            table.put("CounterWheelAppliedVolts", counterWheelAppliedVolts);
            table.put("CounterWheelCurrentAmps", counterWheelCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            // Main flywheel
            flywheelVelocityRPM = table.get("FlywheelVelocityRPM", flywheelVelocityRPM);
            flywheelAppliedVolts = table.get("FlywheelAppliedVolts", flywheelAppliedVolts);
            flywheelCurrentAmps = table.get("FlywheelCurrentAmps", flywheelCurrentAmps);
            flywheelTempCelsius = table.get("FlywheelTempCelsius", flywheelTempCelsius);

            // Individual flywheels
            flywheelAVelocityRPM = table.get("FlywheelAVelocityRPM", flywheelAVelocityRPM);
            flywheelBVelocityRPM = table.get("FlywheelBVelocityRPM", flywheelBVelocityRPM);
            flywheelCVelocityRPM = table.get("FlywheelCVelocityRPM", flywheelCVelocityRPM);

            // Hood
            hoodAngleDegrees = table.get("HoodAngleDegrees", hoodAngleDegrees);
            hoodAppliedVolts = table.get("HoodAppliedVolts", hoodAppliedVolts);
            hoodCurrentAmps = table.get("HoodCurrentAmps", hoodCurrentAmps);

            // Counter-wheel
            counterWheelVelocityRPM = table.get("CounterWheelVelocityRPM", counterWheelVelocityRPM);
            counterWheelAppliedVolts = table.get("CounterWheelAppliedVolts", counterWheelAppliedVolts);
            counterWheelCurrentAmps = table.get("CounterWheelCurrentAmps", counterWheelCurrentAmps);
        }
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
     * @param rpm Target velocity in rotations per minute (0 = stop)
     */
    default void setFlywheelVelocity(double rpm) {}

    /**
     * Sets the hood target angle in degrees.
     * Uses position closed-loop control.
     *
     * @param degrees Target angle in degrees (0 = home/low angle)
     */
    default void setHoodAngle(double degrees) {}

    /**
     * Sets the counter-wheel velocity in RPM (optional).
     *
     * @param rpm Target velocity in rotations per minute
     */
    default void setCounterWheelVelocity(double rpm) {}

    /**
     * Stops all shooter motors.
     */
    default void stop() {}
}
