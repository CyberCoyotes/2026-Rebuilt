package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {

        // Slide — position and velocity needed for MotionMagic and at-target checks
        public double slidePositionRotations = 0.0;
        public double slideVelocityRPS       = 0.0;

        // Sensor data — uncomment as hardware is added
        // public double  intakeDistanceMM = 0.0;
        // public boolean hasGamePiece     = false;
    }

    void updateInputs(IntakeIOInputs inputs);

    void setRollerVoltage(double volts);
    void stopRoller();

    void setSlidePosition(double position);
    void stopSlide();
}