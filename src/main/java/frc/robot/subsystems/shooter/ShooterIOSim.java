package frc.robot.subsystems.shooter;

import frc.robot.Constants;

/**
 * ShooterIOSim - Simulation implementation of ShooterIO.
 *
 * Flywheel is modeled as a first-order filter (spin-up lag).
 * Hood is modeled as a proportional position controller.
 * No real hardware or CAN bus required.
 */
public class ShooterIOSim implements ShooterIO {

    // Flywheel state
    private double flywheelTargetRPM = 0.0;
    private double flywheelRPM = 0.0;

    // Hood state
    private double hoodTargetRotations = 0.0;
    private double hoodPositionRotations = 0.0;

    // Flywheel spin-up time constant (Kraken X60 reaches speed in ~400ms under load)
    private static final double FLYWHEEL_TC_SEC = 0.4;
    // Kraken X60 free-speed used to estimate applied volts
    private static final double FLYWHEEL_FREE_SPEED_RPM = 6000.0;

    // Hood P-gain: position error (rot) → velocity (RPS), clamped to max velocity
    private static final double HOOD_KP = 15.0;
    private static final double HOOD_MAX_VEL_RPS = Constants.Hood.CRUISE_VELOCITY;

    private static final double DT = 0.02;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Flywheel: first-order filter toward target RPM
        double alpha = DT / FLYWHEEL_TC_SEC;
        flywheelRPM += alpha * (flywheelTargetRPM - flywheelRPM);

        // Hood: proportional controller → velocity → integrate position
        double hoodErr = hoodTargetRotations - hoodPositionRotations;
        double hoodVel = Math.max(-HOOD_MAX_VEL_RPS, Math.min(HOOD_MAX_VEL_RPS, hoodErr * HOOD_KP));
        hoodPositionRotations += hoodVel * DT;
        hoodPositionRotations = Math.max(Constants.Hood.MIN_POSE, Math.min(Constants.Hood.MAX_POSE, hoodPositionRotations));

        inputs.flywheelLeaderMotorRPM = flywheelRPM;
        inputs.flywheelLeaderMotorRPS = flywheelRPM / 60.0;
        inputs.flywheelAppliedVolts = (flywheelTargetRPM / FLYWHEEL_FREE_SPEED_RPM) * 12.0;
        inputs.hoodPositionRotations = hoodPositionRotations;
    }

    @Override
    public void updateSlowInputs(ShooterIOInputs inputs) {
        inputs.flywheelCurrentAmps = Math.abs(flywheelRPM) / 100.0;
        inputs.flywheelMaxTempCelsius = 25.0;
        inputs.hoodAppliedVolts = 0.0;
        inputs.hoodCurrentAmps = 0.0;
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        flywheelTargetRPM = rpm;
    }

    @Override
    public void setFlywheelVelocityTorqueFOC(double rpm) {
        flywheelTargetRPM = rpm;
    }

    @Override
    public void stopFlywheels() {
        flywheelTargetRPM = 0.0;
    }

    @Override
    public void setHoodPose(double rawPosition) {
        hoodTargetRotations = rawPosition;
    }

    @Override
    public void setHoodVoltage(double volts) {
        // Open-loop: scale voltage to velocity and integrate directly
        double hoodVel = (volts / 12.0) * HOOD_MAX_VEL_RPS;
        hoodPositionRotations += hoodVel * DT;
        hoodPositionRotations = Math.max(Constants.Hood.MIN_POSE, Math.min(Constants.Hood.MAX_POSE, hoodPositionRotations));
        hoodTargetRotations = hoodPositionRotations;
    }

    @Override
    public void stop() {
        flywheelTargetRPM = 0.0;
        hoodTargetRotations = hoodPositionRotations;
    }
}
