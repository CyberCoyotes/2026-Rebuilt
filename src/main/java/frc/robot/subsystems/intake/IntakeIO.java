package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IntakeIO - Hardware abstraction interface for the intake subsystem.
 *
 * The intake collects game pieces from the ground using:
 * - Roller motor: Spins intake wheels to pull in game pieces
 * - Slide motors: Extends/retracts intake mechanism
 *
 * PATTERN: IO Interface
 * - IntakeIO: Interface defining what intake hardware can do
 * - IntakeIOHardware: Real hardware implementation with CTRE motors
 * - IntakeIOSim: Simulation for testing without hardware
 *
 * @see Constants.Intake for hardware configuration
 * @author @Isaak3
 */
public interface IntakeIO {

    /**
     * IntakeIOInputs - Container for all intake sensor data.
     *
     * This class holds all data we read from the intake hardware each cycle.
     * Uses the Autolog feature for automatic logging and replay
     * Autolog generates 
     */
    @AutoLog
    public static class IntakeIOInputs {
        // ===== Roller Motor Data =====
        /** Roller motor velocity in rotations per second */
        public double rollerVelocityRPS = 0.0;

        /** Roller motor applied voltage */
        public double rollerAppliedVolts = 0.0;

        /** Roller motor supply current in amps */
        public double rollerCurrentAmps = 0.0;

        /** Roller motor temperature in Celsius */
        public double rollerTempCelsius = 0.0;

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

        // Sensor Data
        // distance from nearest thing to intake sensor in mm
        public double intakeDistance = 0.0;

        //is a target detected 
        public boolean intakeTarget = false;

        //distance from nearest thing to indexer in mm
        public double indexerDistance = 0.0;

        public boolean indexerTarget = false;
    }

    /**
     * Updates the inputs object with current hardware readings.
     * Called every robot loop (20ms) by the subsystem.
     *
     * @param inputs Object to populate with current sensor values
     */
    void updateInputs(IntakeIOInputs inputs); //took out default because these methods are fine abstract

    //roller methods
    void setRollerSpeed(double volts);
    double getRollerVolts(); // made doubles because sim can't handle motor-specific StatusSignals
    void stopRoller();

    //slide methods
    void setSlidePosition(double position);
    double getSlidePosition();

    //intake sensor methods
    double getIntakeDistance();
    boolean intakeTargetClose();

    //indexer sensor methods
    double getIndexerDistance();
    boolean indexerTargetClose();

    //multi-hardware methods
    void toRestingState();
    void isJammed();

}
