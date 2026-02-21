package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    /**
     * IntakeIOInputs - Container for all intake sensor data.
     *
     * This class holds all data we read from the intake hardware each cycle.
     * Uses the Autolog feature for automatic logging and replay.
     */
    @AutoLog
    public static class IntakeIOInputs {

        // Slide — position needed for MotionMagic and at-target checks
        public double slidePositionRotations = 0.0;

        /** Slide motor velocity in rotations per second */
        public double slideVelocityRPS = 0.0;

        /** Slide motor applied voltage */
        public double slideAppliedVolts = 0.0;

        /** Slide motor supply current in amps */
        public double slideCurrentAmps = 0.0;

        /** Slide motor temperature in Celsius */
        public double slideTempCelsius = 0.0;

        // // Sensor Data
        // // distance from nearest thing to intake sensor in mm
        // public double intakeDistance = 0.0;

        // // is a target detected
        // public boolean intakeTarget = false;
    }

    /**
     * Updates the inputs object with current hardware readings.
     * Called every robot loop (20ms) by the subsystem.
     *
     * @param inputs Object to populate with current sensor values
     */
    void updateInputs(IntakeIOInputs inputs);

    // ===== Roller methods =====
    void setRollerVoltage(double volts);
    
    double getRollerVoltage();

    void stopRoller();

    
    // ===== Slide methods =====
    /** Position control via MotionMagic — currently commented out in subsystem */
    void setSlidePosition(double position);

    /**
     * Voltage control for slide — use this for extend/retract while MotionMagic is disabled.
     * Positive volts extends, negative volts retracts.
     * Always call stopSlide() when done to prevent the motor running continuously.
     *
     * @param volts Output voltage (-12.0 to 12.0)
     */
    void setSlideVoltage(double volts);

    double getSlidePosition();

    // ===== Intake sensor methods =====
    // double getIntakeDistance();
    // boolean intakeTargetClose();
}