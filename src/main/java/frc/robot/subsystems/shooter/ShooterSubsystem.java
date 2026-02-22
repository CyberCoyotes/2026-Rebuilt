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
 * - STANDBY: Flywheel spinning at STANDBY_RPM — always active during match
 * - READY: Flywheel and hood at preset targets, ready to shoot
 * - PASS: Passing shot at PASS_RPM, hood at PASS_HOOD
 * - EJECT: Flywheel reverse at EJECT_RPM for clearing jams — velocity-gated
 *
 * SHOT FLOW:
 * 2. Driver presses a preset button (A/X/B) — silently updates target RPM and
 * hood angle
 * 3. Driver holds shoot trigger — transitions to READY, hood moves, waits until
 * up to speed, feeds
 * 4. Driver releases trigger — returns to STANDBY at STANDBY_RPM
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

    // == Constants ======
     // --- Flywheel ---
    private static final double MAX_FLYWHEEL_RPM = 6380.0;
    private static final double IDLE_RPM = 0;

    /** TODO tune RPMs for flywheel without excessive current draw
     * Add an end of line comment `Tuned` when each is verified */
    private static final double STANDBY_RPM = 2240; //
    private static final double CLOSE_RPM   = 2750; //
    private static final double FAR_RPM     = 3000; //
    private static final double PASS_RPM    = 4000; //

    /** Reverse RPM for jam clearing. Only reached through eject(), which gates on EJECT_MAX_ENTRY_RPM. */
    private static final double EJECT_RPM = -1500;
    /** Maximum forward flywheel RPM at which EJECT is safe to enter. Prevents violent reversal. */
    private static final double EJECT_MAX_ENTRY_RPM = 500.0;

    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.10; // TODO Consider a tighter tolerance than 10%

     // --- Hood (Kraken rotational positions) ---
    public static final double MIN_HOOD_POSE_ROT  = 0.0;  // Mechanical limit, validate in configs limit
    public static final double MAX_HOOD_POSE_ROT  = 9.14; // Mechanical limit, validate in configs limit
    public static final double HOOD_POSE_TOLERANCE = 0.25; // TODO Tune tolerance based on testing — consider a tighter tolerance than 0.25 rotations

    /** TODO tune Hood rotation position values from Kraken encoder for each shot
     * Consider using WCP Encoder
     * Add an end of line comment `Tuned` when each is verified */
    public static final double CLOSE_HOOD  = 0.00; //
    public static final double TOWER_HOOD  = 4.42; //
    public static final double TRENCH_HOOD = 5.00; //
    public static final double PASS_HOOD   = 5.50; //
    public static final double FAR_HOOD    = 6.00; //

     // --- Testing Increments ---
    public static final double HOOD_TEST_INCREMENT         = 0.2;
    public static final double FLYWHEEL_TEST_INCREMENT_RPM = 100.0;


    // ==== Hardware Interface ====
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // ==== Dashboard Publishers (NetworkTables) ====
    private final NetworkTable shooterTable;
    private final StringPublisher  statePublisher;
    private final BooleanPublisher readyPublisher;
    private final DoublePublisher  flywheelRpmPublisher;
    private final DoublePublisher  targetRpmPublisher;
    private final DoublePublisher  flywheelRpsPublisher;
    private final DoublePublisher  flywheelErrorPublisher;
    private final DoublePublisher  hoodRotationsPublisher;
    private final DoublePublisher  targetHoodRotationsPublisher;
    private final DoublePublisher  hoodErrorPublisher;
    private final BooleanPublisher hoodAtPosePublisher;
    private final DoublePublisher  flywheelVoltsPublisher;
    private final DoublePublisher  throughBorePositionPublisher;
    private final BooleanPublisher throughBoreConnectedPublisher;

    // ==== State ====
    private ShooterState currentState     = ShooterState.IDLE;
    private String currentStateString     = ShooterState.IDLE.toString();
    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot      = 0.0;

    // Slow publish divider
    private int periodicCounter = 0;

    // ==== CONSTRUCTOR ====
    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        shooterTable = inst.getTable("Shooter");

        statePublisher               = shooterTable.getStringTopic("State").publish();
        readyPublisher               = shooterTable.getBooleanTopic("IsReady").publish();
        flywheelRpmPublisher         = shooterTable.getDoubleTopic("FlywheelRPM").publish();
        targetRpmPublisher           = shooterTable.getDoubleTopic("TargetFlywheelRPM").publish();
        flywheelRpsPublisher         = shooterTable.getDoubleTopic("FlywheelMotorRPS").publish();
        flywheelErrorPublisher       = shooterTable.getDoubleTopic("FlywheelError").publish();
        hoodRotationsPublisher       = shooterTable.getDoubleTopic("HoodRotations").publish();
        targetHoodRotationsPublisher = shooterTable.getDoubleTopic("TargetHoodRotations").publish();
        hoodErrorPublisher           = shooterTable.getDoubleTopic("HoodError").publish();
        hoodAtPosePublisher          = shooterTable.getBooleanTopic("HoodAtPose").publish();
        flywheelVoltsPublisher       = shooterTable.getDoubleTopic("FlywheelAppliedVolts").publish();
        throughBorePositionPublisher = shooterTable.getDoubleTopic("ThroughBorePosition").publish();
        throughBoreConnectedPublisher = shooterTable.getBooleanTopic("ThroughBoreConnected").publish();
    }

    // ==== State Machine ====
    public enum ShooterState {
        IDLE,    // Motors off — only used on explicit stop, not during normal match play
        STANDBY, // Flywheel spinning at STANDBY_RPM; ready to ramp to target on demand
        READY,   // Flywheel and hood at preset targets, ready to shoot
        PASS,    // Passing shot at PASS_RPM, hood at PASS_HOOD
        EJECT    // Jam clearing: flywheel at EJECT_RPM (reverse), hood at MIN_POSE — velocity-gated
    }

    // ==== Periodic ====
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
        flywheelRpsPublisher.set(inputs.flywheelLeaderMotorRPS);
        flywheelErrorPublisher.set(targetFlywheelMotorRPM - inputs.flywheelLeaderMotorRPM);
        hoodRotationsPublisher.set(inputs.hoodPositionRotations);
        targetHoodRotationsPublisher.set(targetHoodPoseRot);
        hoodErrorPublisher.set(targetHoodPoseRot - inputs.hoodPositionRotations);
        hoodAtPosePublisher.set(isHoodAtPose());
        flywheelVoltsPublisher.set(inputs.flywheelAppliedVolts);
        throughBorePositionPublisher.set(inputs.hoodThroughBorePositionRotations);
        throughBoreConnectedPublisher.set(inputs.hoodThroughBoreConnected);
    }

    // =====STATE MACHINE=====
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                break;
            case STANDBY:
                break;
            case READY:
                break;
            case PASS:
                break;
            case EJECT:
                break;
        }
    }

    private void setState(ShooterState newState) {
        if (currentState == newState)
            return;

        currentState = newState;
        currentStateString = newState.toString();

        switch (newState) {
            case IDLE:
                io.stopFlywheels();
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case STANDBY:
                io.setFlywheelVelocity(STANDBY_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case READY:
                io.setFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;

            case PASS:
                io.setFlywheelVelocity(PASS_RPM);
                io.setHoodPose(PASS_HOOD);
                break;

            case EJECT:
                io.setFlywheelVelocity(EJECT_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;
        }
    }

    // =========================================================================
    // PUBLIC COMMAND METHODS
    // =========================================================================

    /** Full stop. Not used during normal match play — call returnToStandby() after a shot instead. */
    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    /**
     * Called after a shot is complete to reset the shooter.
     * TODO: Change to setState(ShooterState.STANDBY) once standby spin logic is validated.
     */
    public void returnToStandby() {
        setState(ShooterState.IDLE);
    }

    /**
     * Transitions to READY, ramping flywheel to target and moving hood to target angle.
     */
    public void prepareToShoot() {
        setState(ShooterState.READY);
    }

    /** Sets shooter to passing mode. */
    public void pass() {
        setState(ShooterState.PASS);
    }

    /**
     * Starts ejecting to clear jams.
     * Blocked if flywheel is above EJECT_MAX_ENTRY_RPM to prevent violent reversal.
     */
    public void eject() {
        if (Math.abs(getCurrentVelocityRPM()) > EJECT_MAX_ENTRY_RPM) {
            return; // Flywheel spinning too fast to safely reverse — refuse eject
        }
        setState(ShooterState.EJECT);
    }

    // ===== Silent Preset Setters =====

    /**
     * Silently sets close shot targets. No motor movement until shoot trigger is pressed.
     */
    public void setCloseShotPreset() {
        targetFlywheelMotorRPM = CLOSE_RPM;
        targetHoodPoseRot = CLOSE_HOOD;
    }

    /**
     * Silently sets far shot targets. No motor movement until shoot trigger is pressed.
     */
    public void setFarShotPreset() {
        targetFlywheelMotorRPM = FAR_RPM;
        targetHoodPoseRot = FAR_HOOD;
    }

    /**
     * Silently sets pass shot targets. No motor movement until shoot trigger is pressed.
     */
    public void setPassShotPreset() {
        targetFlywheelMotorRPM = PASS_RPM;
        targetHoodPoseRot = PASS_HOOD;
    }

    // ===== Target Setters =====

    /** Sets target flywheel velocity (forward only — clamped to MAX_FLYWHEEL_RPM). Does NOT change state. */
    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), MAX_FLYWHEEL_RPM);
    }

    /**
     * Sets target hood pose in rotations. Clamped to valid range. Does NOT change state.
     */
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

    /** Returns true if total flywheel current is too high. */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0; // TODO: Tune threshold
    }

    // ── Commands ───────────────────────────────────────────────────────────────

    /** Spins up flywheel to pre-rev speed. */
    public Command spinUpCommand() {
        return Commands.runOnce(this::spinup, this).withName("SpinUp");
    }

    /** Returns shooter to idle. */
    public Command idleCommand() {
        return Commands.runOnce(this::setIdle, this).withName("ShooterIdle");
    }

    /** Test command: ramps flywheel to targetRPM and stops on release. */
    public Command flywheelRampTest(double targetRPM) {
        return Commands.startEnd(
            () -> {
                targetFlywheelMotorRPM = Math.min(Math.abs(targetRPM), MAX_FLYWHEEL_RPM);
                io.setFlywheelVelocity(targetFlywheelMotorRPM);
            },
            this::setIdle,
            this
        ).withName("FlywheelRampTest");
    }

    public ShooterState getState() {
        return currentState;
    }

    public double getCurrentVelocityRPM() {
        return inputs.flywheelLeaderMotorRPM;
    }

    public double getCurrentHoodPose() {
        return inputs.hoodPositionRotations;
    }

    public double getTargetVelocityRPM() {
        return targetFlywheelMotorRPM;
    }

    public double getTargetHoodPose() {
        return targetHoodPoseRot;
    }

    // ==== Vision ====

    /**
     * Updates shooter targets based on distance to target.
     * Linear interpolation between close and far shots.
     * Does NOT change state — call prepareToShoot() after.
     */
    public void updateFromDistance(double distanceMeters) {
        double distance = Math.max(1.0, Math.min(5.0, distanceMeters));
        double t = (distance - 1.0) / (5.0 - 1.0);
        double velocity = CLOSE_RPM + t * (FAR_RPM - CLOSE_RPM);
        double pose = CLOSE_HOOD + t * (FAR_HOOD - CLOSE_HOOD);
        setTargetVelocity(velocity);
        setTargetHoodPose(pose);
    }

    // ==== BACKWARD COMPATIBILITY ====

    public void spinup() {
        setState(ShooterState.STANDBY);
    }

    public void closeShot() {
        setCloseShotPreset();
        prepareToShoot();
    }

    public void farShot() {
        setFarShotPreset();
        prepareToShoot();
    }
}
