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

/**
 * ShooterSubsystem - Main shooter subsystem with state machine control.
 *
 * STATE MACHINE:
 * - IDLE: All motors off, hood at home position (not used during match)
 * - SPINUP: Flywheel spinning at IDLE_RPM — always active during match
 * - READY: Flywheel and hood at preset targets, ready to shoot
 * - PASS: Passing shot at reduced velocity, hood at max
 * - EJECT: Flywheel reverse for clearing jams
 *
 * SHOT FLOW:
 * 1. On boot, shooter enters SPINUP at IDLE_RPM (2000 RPM) automatically
 * 2. Driver presses a preset button (A/X/B) — silently updates target RPM and hood angle
 * 3. Driver holds shoot trigger — transitions to READY, hood moves, waits until up to speed, feeds
 * 4. Driver releases trigger — returns to SPINUP at IDLE_RPM
 *
 * FAR SHOT FLOW (X + RT):
 * - X silently arms far shot and sets isFarShotArmed = true
 * - RT routes to FarShotCommand instead of shootAtCurrentTarget
 * - FarShotCommand continuously updates hood via updateHoodForDistance()
 * - On release, isFarShotArmed is cleared and shooter returns to SPINUP
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */
public class ShooterSubsystem extends SubsystemBase {

    // ===== State Machine =====
    public enum ShooterState {
        IDLE,    // Motors off — only used on explicit stop, not during normal match play
        SPINUP,  // Flywheel spinning at IDLE_RPM, ready to ramp to target on demand
        READY,   // Flywheel and hood at preset targets, ready to shoot
        PASS,    // Passing shot: 50% max velocity, hood at MAX_POSE
        EJECT    // Clearing jams: -50% max velocity, hood at MIN_POSE
    }

    // ===== Hardware Interface =====
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // ===== NetworkTables Publishers for Elastic Dashboard =====
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

    // ===== State =====
    private ShooterState currentState = ShooterState.IDLE;
    private String currentStateString = ShooterState.IDLE.toString();
    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot = MIN_HOOD_POSE_ROT;

    /** True when X (far shot) was the last preset armed. Routes RT to FarShotCommand. */
    private boolean isFarShotArmed = false;

    // ===== Periodic Cycle Counter =====
    private int periodicCounter = 0;

    // ===== Constants =====

    /** Maximum flywheel velocity (RPM) */
    private static final double MAX_FLYWHEEL_MOTOR_RPM = 6000.0;

    /** Idle flywheel velocity (RPM) — always spinning during match for quick response */
    public static final double IDLE_RPM = 2000.0;

    /** Minimum hood pose (rotations) */
    public static final double MIN_HOOD_POSE_ROT = 0.0;

    /** Maximum hood pose (rotations) */
    public static final double MAX_HOOD_POSE_ROT = 9.14;

    /** Flywheel velocity tolerance (10%) */
    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.10;

    /** Hood pose tolerance (rotations) */
    public static final double HOOD_POSE_TOLERANCE = 0.25;

    /** Increment for manual hood adjustment via bumper buttons (rotations) */
    public static final double HOOD_TEST_INCREMENT = 0.5;

    // ===== State-Specific Constants =====

    /** PASS: Flywheel velocity (50% of max) */
    private static final double PASS_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * 0.50;

    /** EJECT: Flywheel velocity (-50% of max) */
    private static final double EJECT_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * -0.50;

    // ===== Shooting Presets =====

    /** Close shot */
    public static final double CLOSE_SHOT_RPM = 2750.0;  // TODO: Tune
    public static final double CLOSE_SHOT_HOOD = 0.0;     // TODO: Tune

    /** Far shot — used as interpolation endpoint in FarShotCommand */
    public static final double FAR_SHOT_RPM = 3000.0;                    // TODO: Tune
    public static final double FAR_SHOT_HOOD = MAX_HOOD_POSE_ROT * 0.5;  // TODO: Tune

    /** Pass shot */
    public static final double PASS_SHOT_RPM = 4000.0;                                          // TODO: Tune
    public static final double PASS_SHOT_HOOD = MAX_HOOD_POSE_ROT - (0.10 * MAX_HOOD_POSE_ROT); // TODO: Tune

    /** Default flywheel velocity testing increment (RPM) */
    public static final double FLYWHEEL_TEST_INCREMENT_RPM = 100.0;

    /** Default target RPM for flywheel ramp-up testing */
    public static final double RAMP_TEST_TARGET_RPM = 4000.0;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

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

        // Boot into SPINUP at idle RPM — flywheel is always warm during match
        // Default targets to close shot until driver selects a preset
        targetFlywheelMotorRPM = CLOSE_SHOT_RPM;
        targetHoodPoseRot = CLOSE_SHOT_HOOD;
        setState(ShooterState.SPINUP);
    }

    // =========================================================================
    // PERIODIC
    // =========================================================================

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

    // =========================================================================
    // STATE MACHINE
    // =========================================================================

    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:   break;
            case SPINUP: break;
            case READY:  break;
            case PASS:   break;
            case EJECT:  break;
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

    // =========================================================================
    // PUBLIC COMMAND METHODS
    // =========================================================================

    /** Full stop. Not used during normal match play — returns to SPINUP instead. */
    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    /**
     * Returns shooter to SPINUP at IDLE_RPM.
     * Call this after a shot is complete to keep the flywheel warm.
     * Also clears the far shot armed flag.
     */
    public void returnToIdle() {
        isFarShotArmed = false;
        setState(ShooterState.SPINUP);
    }

    /** Transitions to READY, ramping flywheel to target and moving hood to target angle. */
    public void prepareToShoot() {
        setState(ShooterState.READY);
    }

    /** Sets shooter to passing mode. */
    public void pass() {
        setState(ShooterState.PASS);
    }

    /** Starts ejecting to clear jams. */
    public void eject() {
        setState(ShooterState.EJECT);
    }

    // ===== Silent Preset Setters =====

    /**
     * Silently sets close shot targets. No motor movement until shoot trigger is pressed.
     * Clears the far shot armed flag.
     */
    public void setCloseShotPreset() {
        isFarShotArmed = false;
        targetFlywheelMotorRPM = CLOSE_SHOT_RPM;
        targetHoodPoseRot = CLOSE_SHOT_HOOD;
    }

    /**
     * Silently sets far shot targets. No motor movement until shoot trigger is pressed.
     * Sets isFarShotArmed = true so RT routes to FarShotCommand.
     */
    public void setFarShotPreset() {
        isFarShotArmed = true;
        targetFlywheelMotorRPM = FAR_SHOT_RPM;
        targetHoodPoseRot = FAR_SHOT_HOOD;
    }

    /**
     * Silently sets pass shot targets. No motor movement until shoot trigger is pressed.
     * Clears the far shot armed flag.
     */
    public void setPassShotPreset() {
        isFarShotArmed = false;
        targetFlywheelMotorRPM = PASS_SHOT_RPM;
        targetHoodPoseRot = PASS_SHOT_HOOD;
    }

    // ===== Target Setters =====

    /** Sets target flywheel velocity. Does NOT change state. */
    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), MAX_FLYWHEEL_MOTOR_RPM);
    }

    /** Sets target hood pose in rotations. Clamped to valid range. Does NOT change state. */
    public void setTargetHoodPose(double rotations) {
        targetHoodPoseRot = Math.max(MIN_HOOD_POSE_ROT, Math.min(MAX_HOOD_POSE_ROT, rotations));
    }

    /** Adjusts target flywheel velocity by a delta. */
    public void adjustTargetVelocity(double deltaRPM) {
        setTargetVelocity(targetFlywheelMotorRPM + deltaRPM);
    }

    /**
     * Adjusts target hood pose by a delta in rotations.
     * If already in READY state, commands hood to move immediately.
     */
    public void adjustTargetHoodPose(double deltaRotations) {
        setTargetHoodPose(targetHoodPoseRot + deltaRotations);
        if (currentState == ShooterState.READY) {
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    /**
     * Updates hood position directly while in READY state.
     * Used by FarShotCommand to continuously adjust hood based on live distance.
     * Only takes effect if currently in READY state — otherwise silently updates target.
     *
     * @param rotations Target hood position in rotations
     */
    public void updateHoodForDistance(double rotations) {
        setTargetHoodPose(rotations);
        if (currentState == ShooterState.READY) {
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    // =========================================================================
    // STATUS QUERIES
    // =========================================================================

    /** Returns true if the far shot preset is currently armed (X was last pressed). */
    public boolean isFarShotArmed() {
        return isFarShotArmed;
    }

    /** Returns true if in READY state with flywheel and hood at targets. */
    public boolean isReady() {
        return currentState == ShooterState.READY && isFlywheelAtVelocity() && isHoodAtPose();
    }

    /** Returns true if in PASS state with flywheel and hood at targets. */
    public boolean isPassReady() {
        return currentState == ShooterState.PASS && isFlywheelAtVelocity() && isHoodAtPose();
    }

    /** Returns true if flywheel is at target velocity within tolerance. */
    public boolean isFlywheelAtVelocity() {
        if (Math.abs(targetFlywheelMotorRPM) < 1.0) {
            return Math.abs(inputs.flywheelLeaderMotorRPM) < 50.0;
        }
        double tolerance = Math.abs(targetFlywheelMotorRPM) * FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    /** Returns true if hood is at target pose within tolerance. */
    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodPositionRotations - targetHoodPoseRot) < HOOD_POSE_TOLERANCE;
    }

    /** Gets current flywheel velocity error (target - actual). */
    public double getFlywheelError() {
        return targetFlywheelMotorRPM - inputs.flywheelLeaderMotorRPM;
    }

    /** Gets current hood pose error (target - actual). */
    public double getHoodError() {
        return targetHoodPoseRot - inputs.hoodPositionRotations;
    }

    public ShooterState getState() { return currentState; }
    public double getCurrentVelocityRPM() { return inputs.flywheelLeaderMotorRPM; }
    public double getCurrentHoodPose() { return inputs.hoodPositionRotations; }
    public double getTargetVelocityRPM() { return targetFlywheelMotorRPM; }
    public double getTargetHoodPose() { return targetHoodPoseRot; }

    /** Returns true if total flywheel current is too high. */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0; // TODO: Tune threshold
    }

    // =========================================================================
    // VISION INTEGRATION
    // =========================================================================

    /**
     * Updates shooter targets based on distance to target.
     * Linear interpolation between close and far shots.
     * Does NOT change state — call prepareToShoot() after.
     */
    public void updateFromDistance(double distanceMeters) {
        double distance = Math.max(1.0, Math.min(5.0, distanceMeters));
        double t = (distance - 1.0) / (5.0 - 1.0);
        double velocity = CLOSE_SHOT_RPM + t * (FAR_SHOT_RPM - CLOSE_SHOT_RPM);
        double pose = CLOSE_SHOT_HOOD + t * (FAR_SHOT_HOOD - CLOSE_SHOT_HOOD);
        setTargetVelocity(velocity);
        setTargetHoodPose(pose);
    }

    // =========================================================================
    // FLYWHEEL RAMP TESTING
    // =========================================================================

    public Command flywheelRampTest(double targetRPM) {
        return Commands.startEnd(
                () -> {
                    setTargetVelocity(targetRPM);
                    prepareToShoot();
                },
                this::returnToIdle,
                this).withName("FlywheelRampTest");
    }

    // =========================================================================
    // BACKWARD COMPATIBILITY
    // =========================================================================

    public void spinup() { setState(ShooterState.SPINUP); }
    public void closeShot() { setCloseShotPreset(); prepareToShoot(); }
    public void farShot() { setFarShotPreset(); prepareToShoot(); }
}