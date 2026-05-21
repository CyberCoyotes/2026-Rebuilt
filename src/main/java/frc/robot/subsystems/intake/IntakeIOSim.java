package frc.robot.subsystems.intake;

import frc.robot.Constants;

/**
 * IntakeIOSim - Simulation implementation of IntakeIO.
 *
 * Models the slide as a proportional position controller and the roller as a
 * first-order velocity filter. No real hardware or CAN bus required.
 */
public class IntakeIOSim implements IntakeIO {

    // Slide state
    private double slidePositionRotations = 0.0;
    private double slideTargetPosition = 0.0;

    // Roller state
    private double rollerTargetRPS = 0.0;
    private double rollerVelocityRPS = 0.0;

    // Slide P-gain: position error (rot) → velocity (RPS), clamped to cruise velocity
    private static final double SLIDE_KP = 10.0;
    private static final double SLIDE_MAX_VEL_RPS = Constants.Intake.SLIDE_MM_CRUISE_VELOCITY;

    // Roller first-order time constant (seconds to approach setpoint)
    private static final double ROLLER_TC_SEC = 0.1;

    private static final double DT = 0.02;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Slide: proportional controller → velocity → integrate position
        double posError = slideTargetPosition - slidePositionRotations;
        double slideVelRPS = Math.max(-SLIDE_MAX_VEL_RPS, Math.min(SLIDE_MAX_VEL_RPS, posError * SLIDE_KP));
        slidePositionRotations += slideVelRPS * DT;
        // Clamp to physical travel limits (mirrors hardware soft limits)
        slidePositionRotations = Math.max(0.0, Math.min(Constants.Intake.SLIDE_MAX_POS, slidePositionRotations));

        // Roller: first-order filter toward target velocity
        double alpha = DT / ROLLER_TC_SEC;
        rollerVelocityRPS += alpha * (rollerTargetRPS - rollerVelocityRPS);

        inputs.slidePositionRotations = slidePositionRotations;
        inputs.slideVelocityRPS = slideVelRPS;
    }

    @Override
    public void setRollerVelocity(double rps) {
        rollerTargetRPS = rps;
    }

    @Override
    public void stopRoller() {
        rollerTargetRPS = 0.0;
    }

    @Override
    public void setSlidePosition(double position) {
        slideTargetPosition = position;
    }

    @Override
    public void setSlidePositionSlow(double position) {
        slideTargetPosition = position;
    }

    @Override
    public void stopSlide() {
        slideTargetPosition = slidePositionRotations;
    }

    @Override
    public void resetSlideEncoder() {
        slidePositionRotations = 0.0;
        slideTargetPosition = 0.0;
    }
}
