package frc.robot.subsystems.intake2;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * IntakeIO - Hardware abstraction interface for the intake subsystem.
 *
 * The intake collects game pieces from the ground using:
 * - 1x Rotator motor: Spins intake wheels to pull in game pieces
 * - 1x Slide motor: Extends/retracts intake mechanism
 *
 * HARDWARE:
 * - 1x TalonFX (rotator) - percent output control
 * - 1x TalonFX (slide) - position control
 *
 * PATTERN: IO Interface
 * - IntakeIO: Interface defining what intake hardware can do
 * - IntakeIOTalonFX: Real hardware implementation with CTRE motors
 * - IntakeIOSim: Simulation for testing without hardware
 *
 * @author @Isaak3
 */
public interface IntakeIO {

    /**
     * IntakeIOInputs - Container for all intake sensor data.
     *
     * This class holds all data we read from the intake hardware each cycle.
     * Implements LoggableInputs for automatic AdvantageKit logging and replay.
     */
    class IntakeIOInputs implements LoggableInputs {
        // ===== Rotator Motor Data =====
        /** Rotator motor velocity in rotations per second */
        public double rotatorVelocityRPS = 0.0;

        /** Rotator motor applied voltage */
        public double rotatorAppliedVolts = 0.0;

        /** Rotator motor supply current in amps */
        public double rotatorCurrentAmps = 0.0;

        /** Rotator motor temperature in Celsius */
        public double rotatorTempCelsius = 0.0;

        // ===== Slide Motor Data =====
        /** Slide position in rotations (0 = retracted) */
        public double slidePositionRotations = 0.0;

        /** Slide motor velocity in rotations per second */
        public double slideVelocityRPS = 0.0;

        /** Slide motor applied voltage */
        public double slideAppliedVolts = 0.0;

        /** Slide motor supply current in amps */
        public double slideCurrentAmps = 0.0;

        /** Slide motor temperature in Celsius */
        public double slideTempCelsius = 0.0;

        @Override
        public void toLog(LogTable table) {
            // Rotator motor
            table.put("RotatorVelocityRPS", rotatorVelocityRPS);
            table.put("RotatorAppliedVolts", rotatorAppliedVolts);
            table.put("RotatorCurrentAmps", rotatorCurrentAmps);
            table.put("RotatorTempCelsius", rotatorTempCelsius);

            // Slide motor
            table.put("SlidePositionRotations", slidePositionRotations);
            table.put("SlideVelocityRPS", slideVelocityRPS);
            table.put("SlideAppliedVolts", slideAppliedVolts);
            table.put("SlideCurrentAmps", slideCurrentAmps);
            table.put("SlideTempCelsius", slideTempCelsius);
        }

        @Override
        public void fromLog(LogTable table) {
            // Rotator motor
            rotatorVelocityRPS = table.get("RotatorVelocityRPS", rotatorVelocityRPS);
            rotatorAppliedVolts = table.get("RotatorAppliedVolts", rotatorAppliedVolts);
            rotatorCurrentAmps = table.get("RotatorCurrentAmps", rotatorCurrentAmps);
            rotatorTempCelsius = table.get("RotatorTempCelsius", rotatorTempCelsius);

            // Slide motor
            slidePositionRotations = table.get("SlidePositionRotations", slidePositionRotations);
            slideVelocityRPS = table.get("SlideVelocityRPS", slideVelocityRPS);
            slideAppliedVolts = table.get("SlideAppliedVolts", slideAppliedVolts);
            slideCurrentAmps = table.get("SlideCurrentAmps", slideCurrentAmps);
            slideTempCelsius = table.get("SlideTempCelsius", slideTempCelsius);
        }
    }

    /**
     * Updates the inputs object with current hardware readings.
     * Called every robot loop (20ms) by the subsystem.
     *
     * @param inputs Object to populate with current sensor values
     */
    default void updateInputs(IntakeIOInputs inputs) {}

    /**
     * Sets the rotator motor output (intake wheels).
     *
     * @param percent Motor output from -1.0 to 1.0 (positive = intake)
     */
    default void setRotator(double percent) {}

    /**
     * Sets the slide position (extend/retract).
     *
     * @param rotations Target position in motor rotations (0 = retracted)
     */
    default void setSlidePosition(double rotations) {}

    /**
     * Stops all intake motors.
     */
    default void stop() {}

    /**
     * Zeros the slide encoder at the current position.
     * Call this when the intake is manually moved to the retracted position.
     */
    default void zeroSlide() {}
}
