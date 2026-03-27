package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {

    /**
     * IntakeIOInputs - Container for all intake sensor data.
     *
     * This class holds all data we read from the intake hardware each cycle.
     * Uses the Autolog feature for automatic logging and replay.
     */
    public static class IntakeIOInputs implements LoggableInputs{

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

        @Override
        public void toLog(LogTable table) {
            table.put("Slide Position Rotations", slidePositionRotations);
            table.put("Slide Velocity RPS", slideVelocityRPS);
            table.put("Slide Applied Volts", slideAppliedVolts);
            table.put("Slide Current Amps", slideCurrentAmps);
            table.put("Slide Temp Celsius", slideTempCelsius);
        }

        @Override
        public void fromLog(LogTable table) {
            slidePositionRotations = table.get("Slide Position Rotations", slidePositionRotations);
            slideVelocityRPS = table.get("Slide Velocity RPS", slideVelocityRPS);
            slideAppliedVolts = table.get("Slide Applied Volts", slideAppliedVolts);
            slideCurrentAmps = table.get("Slide Current Amps", slideCurrentAmps);
            slideTempCelsius = table.get("Slide Temp Celsius", slideTempCelsius);
        }
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