package frc.robot.subsystems.shooter;

// import org.littletonrobotics.junction.Logger;

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

    // ── State Machine ──────────────────────────────────────────────────────────
    public enum ShooterState {
        IDLE,    // Flywheel stopped, hood at MIN_POSE
        SPINUP,  // Pre-rev flywheel, hood at mid position
        READY,   // Flywheel and hood at targets, ready to shoot
        PASS,    // Passing shot: 50% max velocity, hood at MAX_POSE
        EJECT    // Clearing jams: -50% max velocity, hood at MIN_POSE
    }

    // ── Hardware Interface ─────────────────────────────────────────────────────
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // ── Dashboard Publishers ───────────────────────────────────────────────────
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

    // ── State ──────────────────────────────────────────────────────────────────
    private ShooterState currentState = ShooterState.IDLE;
    private String currentStateString = ShooterState.IDLE.toString();
    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot = MIN_HOOD_POSE_ROT;

    // ── Cycle Counter (throttle slow reads + NT publishing to 10Hz) ────────────
    private int periodicCounter = 0;

    // ── Constants ──────────────────────────────────────────────────────────────
    private static final double MAX_FLYWHEEL_MOTOR_RPM    = 6380.0;
    private static final double MIN_HOOD_POSE_ROT         = 0.0;
    private static final double MAX_HOOD_POSE_ROT         = 9.14;

    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.10; // TODO: Tighten for competition
    private static final double HOOD_POSE_TOLERANCE         = 0.10; // TODO: Tune for shot consistency

    public static final double HOOD_TEST_INCREMENT          = 0.5;
    public static final double FLYWHEEL_TEST_INCREMENT_RPM  = 100.0;
    public static final double RAMP_TEST_TARGET_RPM         = 2800.0; // TODO: Verify

    // State-specific velocities
    private static final double PASS_VELOCITY_RPM  = MAX_FLYWHEEL_MOTOR_RPM * 0.50;
    private static final double EJECT_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * -0.50;
    private static final double SPINUP_RPM         = 0.80 * 1800.0; // 80% of close shot RPM

    // Shooting presets
    public static final double CLOSE_SHOT_RPM  = 1800.0;                       // TODO: Tune
    public static final double CLOSE_SHOT_HOOD = 0.0;                          // TODO: Tune
    public static final double FAR_SHOT_RPM    = 2000.0;                       // TODO: Tune
    public static final double FAR_SHOT_HOOD   = MAX_HOOD_POSE_ROT * 0.5;      // TODO: Tune
    public static final double PASS_SHOT_RPM   = 3000.0;                       // TODO: Tune
    public static final double PASS_SHOT_HOOD  = MAX_HOOD_POSE_ROT * 0.90;     // TODO: Tune

    // ── Constructor ────────────────────────────────────────────────────────────
    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        shooterTable = inst.getTable("Shooter");

        statePublisher               = shooterTable.getStringTopic("State").publish();
        readyPublisher               = shooterTable.getBooleanTopic("IsReady").publish();
        flywheelRpmPublisher         = shooterTable.getDoubleTopic("FlywheelLeaderMotorRPM").publish();
        targetRpmPublisher           = shooterTable.getDoubleTopic("TargetFlywheelLeaderMotorRPM").publish();
        flywheelErrorPublisher       = shooterTable.getDoubleTopic("FlywheelError").publish();
        hoodErrorPublisher           = shooterTable.getDoubleTopic("HoodError").publish();
        hoodAtPosePublisher          = shooterTable.getBooleanTopic("HoodAtPose").publish();
        flywheelVoltsPublisher       = shooterTable.getDoubleTopic("FlywheelAppliedVolts").publish();
        throughBorePositionPublisher = shooterTable.getDoubleTopic("ThroughBorePosition").publish();
        throughBoreConnectedPublisher = shooterTable.getBooleanTopic("ThroughBoreConnected").publish();
    }

    // ── Periodic ───────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        updateStateMachine();

        // 10Hz — diagnostic reads and dashboard publishing
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
    }

    // ── State Machine ──────────────────────────────────────────────────────────
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
            case SPINUP:
            case READY:
            case PASS:
            case EJECT:
                break; // IO layer maintains targets; transitions are manual
        }
    }

    private void setState(ShooterState newState) {
        if (currentState == newState) return;

        currentState = newState;
        currentStateString = newState.toString();

        switch (newState) {
            case IDLE:
                io.stopFlywheels();
                targetFlywheelMotorRPM = 0.0;
                targetHoodPoseRot = MIN_HOOD_POSE_ROT;
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case SPINUP:
                targetFlywheelMotorRPM = SPINUP_RPM;
                io.setFlywheelVelocity(SPINUP_RPM);
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
                targetFlywheelMotorRPM = EJECT_VELOCITY_RPM;
                targetHoodPoseRot = MIN_HOOD_POSE_ROT;
                io.setFlywheelVelocity(EJECT_VELOCITY_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;
        }
    }

    // ── State Transitions ──────────────────────────────────────────────────────
    public void setIdle()         { setState(ShooterState.IDLE); }
    public void spinup()          { setState(ShooterState.SPINUP); }
    public void prepareToShoot()  { setState(ShooterState.READY); }
    public void pass()            { setState(ShooterState.PASS); }
    public void eject()           { setState(ShooterState.EJECT); }

    // ── Shooting Presets ───────────────────────────────────────────────────────
    public void closeShot() {
        setTargetVelocity(CLOSE_SHOT_RPM);
        setTargetHoodPose(CLOSE_SHOT_HOOD);
        setState(ShooterState.READY);
    }

    public void farShot() {
        setTargetVelocity(FAR_SHOT_RPM);
        setTargetHoodPose(FAR_SHOT_HOOD);
        setState(ShooterState.READY);
    }

    // ── Target Setters ─────────────────────────────────────────────────────────
    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), MAX_FLYWHEEL_MOTOR_RPM);
    }

    public void setTargetHoodPose(double rotations) {
        targetHoodPoseRot = Math.max(MIN_HOOD_POSE_ROT, Math.min(MAX_HOOD_POSE_ROT, rotations));
    }

    public void adjustTargetVelocity(double deltaRPM) {
        setTargetVelocity(targetFlywheelMotorRPM + deltaRPM);
    }

    public void increaseTargetVelocity() { adjustTargetVelocity(FLYWHEEL_TEST_INCREMENT_RPM); }
    public void decreaseTargetVelocity() { adjustTargetVelocity(-FLYWHEEL_TEST_INCREMENT_RPM); }

    // ── Vision Integration ─────────────────────────────────────────────────────
    public void updateFromDistance(double distanceMeters) {
        // TODO: Replace with proper ballistic calculations
        double distance = Math.max(1.0, Math.min(5.0, distanceMeters));
        double t        = (distance - 1.0) / (5.0 - 1.0);
        setTargetVelocity(CLOSE_SHOT_RPM + t * (FAR_SHOT_RPM - CLOSE_SHOT_RPM));
        setTargetHoodPose(CLOSE_SHOT_HOOD + t * (FAR_SHOT_HOOD - CLOSE_SHOT_HOOD));
    }

    // ── Status Queries ─────────────────────────────────────────────────────────
    public boolean isReady() {
        return currentState == ShooterState.READY && isFlywheelAtVelocity() && isHoodAtPose();
    }

    public boolean isPassReady() {
        return currentState == ShooterState.PASS && isFlywheelAtVelocity() && isHoodAtPose();
    }

    public boolean isFlywheelAtVelocity() {
        if (Math.abs(targetFlywheelMotorRPM) < 1.0) {
            return Math.abs(inputs.flywheelLeaderMotorRPM) < 50.0;
        }
        double tolerance = Math.abs(targetFlywheelMotorRPM) * FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodPositionRotations - targetHoodPoseRot) < HOOD_POSE_TOLERANCE;
    }

    public boolean isOverCurrent()         { return inputs.flywheelCurrentAmps > 150.0; } // TODO: Tune

    public double getFlywheelError()        { return targetFlywheelMotorRPM - inputs.flywheelLeaderMotorRPM; }
    public double getHoodError()            { return targetHoodPoseRot - inputs.hoodPositionRotations; }
    public ShooterState getState()          { return currentState; }
    public double getCurrentVelocityRPM()   { return inputs.flywheelLeaderMotorRPM; }
    public double getCurrentHoodPose()      { return inputs.hoodPositionRotations; }
    public double getTargetVelocityRPM()    { return targetFlywheelMotorRPM; }
    public double getTargetHoodPose()       { return targetHoodPoseRot; }

    // ── Commands ───────────────────────────────────────────────────────────────

    /** Spins up flywheel to pre-rev speed. */
    public Command spinUpCommand() {
        return Commands.runOnce(this::spinup, this).withName("SpinUp");
    }

    /** Returns shooter to idle. */
    public Command idleCommand() {
        return Commands.runOnce(this::setIdle, this).withName("ShooterIdle");
    }

    /** Reverses flywheel for a set duration to clear a jam, then idles. */
    public Command ejectCommand(double durationSeconds) {
        return Commands.sequence(
            Commands.runOnce(this::eject, this),
            Commands.waitSeconds(durationSeconds),
            Commands.runOnce(this::setIdle, this)
        ).withName("EjectShooter");
    }

    /** Configures close shot preset and waits until ready. */
    public Command closeShotCommand() {
        return Commands.sequence(
            Commands.runOnce(this::closeShot, this),
            Commands.waitUntil(this::isReady)
        ).withTimeout(3.0).withName("CloseShot");
    }

    /** Configures far shot preset and waits until ready. */
    public Command farShotCommand() {
        return Commands.sequence(
            Commands.runOnce(this::farShot, this),
            Commands.waitUntil(this::isReady)
        ).withTimeout(3.0).withName("FarShot");
    }

    /** Configures pass preset and waits until ready. */
    public Command passCommand() {
        return Commands.sequence(
            Commands.runOnce(this::pass, this),
            Commands.waitUntil(this::isPassReady)
        ).withTimeout(3.0).withName("Pass");
    }

    /** Sets specific targets and waits until ready. */
    public Command prepareToShootCommand(double velocityRPM, double hoodAngleDegrees) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                setTargetVelocity(velocityRPM);
                setTargetHoodPose(hoodAngleDegrees);
                prepareToShoot();
            }, this),
            Commands.waitUntil(this::isReady)
        ).withTimeout(3.0).withName("PrepareToShoot");
    }

    /** Waits until shooter is ready. Useful in parallel command groups. */
    public Command waitUntilReadyCommand() {
        return Commands.waitUntil(this::isReady).withTimeout(3.0).withName("WaitForShooterReady");
    }

    /** Increases target velocity by one increment. */
    public Command increaseTargetVelocityCommand() {
        return Commands.runOnce(this::increaseTargetVelocity, this).withName("IncreaseShooterVelocity");
    }

    /** Decreases target velocity by one increment. */
    public Command decreaseTargetVelocityCommand() {
        return Commands.runOnce(this::decreaseTargetVelocity, this).withName("DecreaseShooterVelocity");
    }

    /**
     * Ramps flywheel to target RPM while held, idles on release.
     * Ramp rate governed by ClosedLoopRamps in motor config.
     */
    public Command flywheelRampTest(double targetRPM) {
        return Commands.startEnd(
            () -> {
                setTargetVelocity(targetRPM);
                prepareToShoot();
            },
            this::setIdle,
            this
        ).withName("FlywheelRampTest");
    }
}