package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem extends SubsystemBase {

    public enum ShooterState {
        IDLE,
        SPINUP,
        READY,
        PASS,
        EJECT
    }

    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    private final NetworkTable shooterTable;
    private final StringPublisher statePublisher;
    private final BooleanPublisher readyPublisher;
    private final DoublePublisher flywheelRpmPublisher;
    private final DoublePublisher targetRpmPublisher;
    private final DoublePublisher flywheelErrorPublisher;
    private final DoublePublisher hoodErrorPublisher;
    private final BooleanPublisher hoodAtPosePublisher;
    private final DoublePublisher flywheelVoltsPublisher;
    private final DoublePublisher throughBorePositionPublisher;
    private final BooleanPublisher throughBoreConnectedPublisher;
    private final BooleanPublisher farShotArmedPublisher;

    private ShooterState currentState = ShooterState.IDLE;
    private String currentStateString = ShooterState.IDLE.toString();

    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot = MIN_HOOD_POSE_ROT;

    private boolean isFarShotArmed = false;

    private int periodicCounter = 0;

    private static final double MAX_FLYWHEEL_MOTOR_RPM = 6000.0;
    public static final double IDLE_RPM = 2000.0;

    public static final double MIN_HOOD_POSE_ROT = 0.0;
    public static final double MAX_HOOD_POSE_ROT = 9.14;

    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.10;
    public static final double HOOD_POSE_TOLERANCE = 0.25;

    public static final double HOOD_TEST_INCREMENT = 0.5;

    private static final double PASS_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * 0.50;
    private static final double EJECT_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * -0.50;

    public static final double CLOSE_SHOT_RPM = 2750.0;
    public static final double CLOSE_SHOT_HOOD = 0.0;

    public static final double FAR_SHOT_RPM = 2750.0;
    public static final double FAR_SHOT_HOOD = MAX_HOOD_POSE_ROT * 0.5;

    public static final double PASS_SHOT_RPM = 4000.0;
    public static final double PASS_SHOT_HOOD = MAX_HOOD_POSE_ROT * 0.9;

    public static final double FLYWHEEL_TEST_INCREMENT_RPM = 100.0;
    public static final double RAMP_TEST_TARGET_RPM = 4000.0;

    public ShooterSubsystem(ShooterIO io) {

        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        shooterTable = inst.getTable("Shooter");

        statePublisher = shooterTable.getStringTopic("State").publish();
        readyPublisher = shooterTable.getBooleanTopic("IsReady").publish();
        flywheelRpmPublisher = shooterTable.getDoubleTopic("FlywheelLeaderMotorRPM").publish();
        targetRpmPublisher = shooterTable.getDoubleTopic("TargetFlywheelLeaderMotorRPM").publish();
        flywheelErrorPublisher = shooterTable.getDoubleTopic("FlywheelError").publish();
        hoodErrorPublisher = shooterTable.getDoubleTopic("HoodError").publish();
        hoodAtPosePublisher = shooterTable.getBooleanTopic("HoodAtPose").publish();
        flywheelVoltsPublisher = shooterTable.getDoubleTopic("FlywheelAppliedVolts").publish();
        throughBorePositionPublisher = shooterTable.getDoubleTopic("ThroughBorePosition").publish();
        throughBoreConnectedPublisher = shooterTable.getBooleanTopic("ThroughBoreConnected").publish();
        farShotArmedPublisher = shooterTable.getBooleanTopic("FarShotArmed").publish();

        targetFlywheelMotorRPM = CLOSE_SHOT_RPM;
        targetHoodPoseRot = CLOSE_SHOT_HOOD;

        setState(ShooterState.SPINUP);
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);

        updateStateMachine();

        if (++periodicCounter % 5 == 0) {
            io.updateSlowInputs(inputs);
            publishToElastic();
        }
    }

    private void publishToElastic() {

        statePublisher.set(currentStateString);
        readyPublisher.set(isReady());
        flywheelRpmPublisher.set(inputs.flywheelLeaderMotorRPM);
        targetRpmPublisher.set(targetFlywheelMotorRPM);
        flywheelErrorPublisher.set(getFlywheelError());
        hoodErrorPublisher.set(getHoodError());
        hoodAtPosePublisher.set(isHoodAtPose());
        flywheelVoltsPublisher.set(inputs.flywheelAppliedVolts);
        throughBorePositionPublisher.set(inputs.hoodThroughBorePositionRotations);
        throughBoreConnectedPublisher.set(inputs.hoodThroughBoreConnected);
        farShotArmedPublisher.set(isFarShotArmed);
    }

    private void updateStateMachine() {

        switch (currentState) {

            case IDLE:
                break;

            case SPINUP:
                io.setFlywheelVelocity(IDLE_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case READY:
                io.setFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;

            case PASS:
                io.setFlywheelVelocity(PASS_VELOCITY_RPM);
                io.setHoodPose(MAX_HOOD_POSE_ROT);
                break;

            case EJECT:
                io.setFlywheelVelocity(EJECT_VELOCITY_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;
        }
    }

    private void setState(ShooterState newState) {

        if (currentState == newState) return;

        currentState = newState;
        currentStateString = newState.toString();

        switch (newState) {

            case IDLE:
                io.stopFlywheels();
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case SPINUP:
                io.setFlywheelVelocity(IDLE_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case READY:
                io.setFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;

            case PASS:
                targetFlywheelMotorRPM = PASS_VELOCITY_RPM;
                targetHoodPoseRot = MAX_HOOD_POSE_ROT;
                io.setFlywheelVelocity(PASS_VELOCITY_RPM);
                io.setHoodPose(MAX_HOOD_POSE_ROT);
                break;

            case EJECT:
                io.setFlywheelVelocity(EJECT_VELOCITY_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;
        }
    }

    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    public void returnToIdle() {
        isFarShotArmed = false;
        setState(ShooterState.SPINUP);
    }

    public void prepareToShoot() {
        setState(ShooterState.READY);
    }

    public void pass() {
        setState(ShooterState.PASS);
    }

    public void eject() {
        setState(ShooterState.EJECT);
    }

    public void setCloseShotPreset() {
        isFarShotArmed = false;
        targetFlywheelMotorRPM = CLOSE_SHOT_RPM;
        targetHoodPoseRot = CLOSE_SHOT_HOOD;
    }

    public void setFarShotPreset() {
        isFarShotArmed = true;
        targetFlywheelMotorRPM = FAR_SHOT_RPM;
        targetHoodPoseRot = FAR_SHOT_HOOD;
    }

    public void setPassShotPreset() {
        isFarShotArmed = false;
        targetFlywheelMotorRPM = PASS_SHOT_RPM;
        targetHoodPoseRot = PASS_SHOT_HOOD;
    }

    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), MAX_FLYWHEEL_MOTOR_RPM);
    }

    public void setTargetHoodPose(double rotations) {
        targetHoodPoseRot = Math.max(MIN_HOOD_POSE_ROT,
                Math.min(MAX_HOOD_POSE_ROT, rotations));
    }

    public void adjustTargetVelocity(double deltaRPM) {
        setTargetVelocity(targetFlywheelMotorRPM + deltaRPM);
    }

    public void adjustTargetHoodPose(double deltaRotations) {

        setTargetHoodPose(targetHoodPoseRot + deltaRotations);

        if (currentState == ShooterState.READY) {
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    public void updateHoodForDistance(double rotations) {

        setTargetHoodPose(rotations);

        if (currentState == ShooterState.READY) {
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    public void updateFlywheelVelocity(double rpm) {

        targetFlywheelMotorRPM =
                Math.min(Math.abs(rpm), MAX_FLYWHEEL_MOTOR_RPM);

        if (currentState == ShooterState.READY) {
            io.setFlywheelVelocity(targetFlywheelMotorRPM);
        }
    }

    public boolean isFarShotArmed() {
        return isFarShotArmed;
    }

    public boolean isReady() {
        return currentState == ShooterState.READY
                && isFlywheelAtVelocity()
                && isHoodAtPose();
    }

    public boolean isPassReady() {
        return currentState == ShooterState.PASS
                && isFlywheelAtVelocity()
                && isHoodAtPose();
    }

    public boolean isFlywheelAtVelocity() {

        if (Math.abs(targetFlywheelMotorRPM) < 1.0) {
            return Math.abs(inputs.flywheelLeaderMotorRPM) < 50.0;
        }

        double tolerance =
                Math.abs(targetFlywheelMotorRPM)
                        * FLYWHEEL_TOLERANCE_PERCENT;

        return Math.abs(inputs.flywheelLeaderMotorRPM
                - targetFlywheelMotorRPM) < tolerance;
    }

    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodPositionRotations
                - targetHoodPoseRot) < HOOD_POSE_TOLERANCE;
    }

    public double getFlywheelError() {
        return targetFlywheelMotorRPM
                - inputs.flywheelLeaderMotorRPM;
    }

    public double getHoodError() {
        return targetHoodPoseRot
                - inputs.hoodPositionRotations;
    }

    public ShooterState getState() { return currentState; }
    public double getCurrentVelocityRPM() { return inputs.flywheelLeaderMotorRPM; }
    public double getCurrentHoodPose() { return inputs.hoodPositionRotations; }
    public double getTargetVelocityRPM() { return targetFlywheelMotorRPM; }
    public double getTargetHoodPose() { return targetHoodPoseRot; }

    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0;
    }

    public void updateFromDistance(double distanceMeters) {

        double distance = Math.max(1.0,
                Math.min(5.0, distanceMeters));

        double t = (distance - 1.0) / (5.0 - 1.0);

        double velocity =
                CLOSE_SHOT_RPM + t *
                (FAR_SHOT_RPM - CLOSE_SHOT_RPM);

        double pose =
                CLOSE_SHOT_HOOD + t *
                (FAR_SHOT_HOOD - CLOSE_SHOT_HOOD);

        setTargetVelocity(velocity);
        setTargetHoodPose(pose);
    }

    public Command flywheelRampTest(double targetRPM) {

        return Commands.startEnd(
                () -> {
                    setTargetVelocity(targetRPM);
                    prepareToShoot();
                },
                this::returnToIdle,
                this
        ).withName("FlywheelRampTest");
    }

    public void spinup() { setState(ShooterState.SPINUP); }
    public void closeShot() { setCloseShotPreset(); prepareToShoot(); }
    public void farShot() { setFarShotPreset(); prepareToShoot(); }
}