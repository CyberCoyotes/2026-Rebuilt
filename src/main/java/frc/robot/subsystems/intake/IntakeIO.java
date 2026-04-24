package frc.robot.subsystems.intake;

public interface IntakeIO {

    /**
     * IntakeIOInputs - Container for all intake sensor data.
     *
     * This class holds all data we read from the intake hardware each cycle.
     */
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
    void setRollerVelocity(double rps);

    void stopRoller();

    
    // ===== Slide methods =====
    /** Position control via MotionMagic */
    void setSlidePosition(double position);
    
    /** Position control via MotionMagic with slower velocity */
    void setSlidePositionSlow(double position);

    void stopSlide();

    // double getSlidePosition();

    // ===== Intake sensor methods =====
    // double getIntakeDistance();
    // boolean intakeTargetClose();
}
