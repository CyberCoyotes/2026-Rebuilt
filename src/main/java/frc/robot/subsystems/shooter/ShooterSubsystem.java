package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

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
 * - IDLE: All motors off, hood at home position
 * - SPINUP: Flywheel accelerating to target, hood moving to target angle
 * - READY: Both flywheel and hood at target, ready to shoot
 * - PASS: Passing shot at reduced velocity, hood at max
 * - EJECT: Flywheel reverse for clearing jams
 *
 * USAGE:
 * 1. Call setTargetVelocity() and setTargetHoodPose() to set desired parameters
 * 2. Call spinup() to start motors (or use a preset like closeShot())
 * 3. Wait for isReady() to return true
 * 4. Signal indexer to feed game piece
 * 5. Call setIdle() when done
 *
 * PERFORMANCE:
 * - CAN reads are split: control-critical every cycle, diagnostics at 10Hz
 * - NT publishing throttled to 10Hz (Elastic can't display faster anyway)
 * - State string cached on transitions to avoid per-cycle allocation
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */
public class ShooterSubsystem extends SubsystemBase {

    // ===== State Machine =====
    public enum ShooterState {
        IDLE,    // Flywheel stopped, hood at MIN_POSE
        SPINUP,  // Pre-rev flywheel, hood at mid position
        READY,   // Flywheel and hood at vision-based targets, ready to shoot
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

    // ===== State =====
    private ShooterState currentState = ShooterState.IDLE;
    private String currentStateString = ShooterState.IDLE.toString(); // cached to avoid per-cycle allocation
    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot = MIN_HOOD_POSE_ROT;

    // ===== Periodic Cycle Counter =====
    // Used to throttle slow CAN reads and NT publishing to 10Hz (every 5th cycle)
    private int periodicCounter = 0;

    // ===== Constants =====

    /** Maximum flywheel velocity (RPM) Free Spin */
    private static final double MAX_FLYWHEEL_MOTOR_RPM = 6380.0;

    /** Minimum hood pose (rotations) */
    private static final double MIN_HOOD_POSE_ROT = 0.0;

    /** Maximum hood pose (rotations) */
    private static final double MAX_HOOD_POSE_ROT = 9.14;

    /**
     * Flywheel velocity tolerance (percentage of target, 0.0 to 1.0)
     * TODO Lower this as needed for more precise control
     * 10% tolerance — forgiving for initial testing, tighten for competition
     */
    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.10;

    /** Hood pose tolerance (rotations) */
    private static final double HOOD_POSE_TOLERANCE = 0.10; // TODO Tune to appropriate value for shot consistency

    /** Testing increment for manual hood adjustment (rotations) */
    public static final double HOOD_TEST_INCREMENT = 0.5;

    // ===== State-Specific Constants =====

    /** PASS: Flywheel velocity (50% of max) */
    private static final double PASS_VELOCITY_PERCENT = 0.50;
    private static final double PASS_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * PASS_VELOCITY_PERCENT;

    /** EJECT: Flywheel velocity (-50% of max, negative for reverse) */
    private static final double EJECT_VELOCITY_PERCENT = -0.50;
    private static final double EJECT_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * EJECT_VELOCITY_PERCENT;

    // ===== Shooting Presets (for READY state) =====

    /** Close shot velocity (RPM) */
    public static final double CLOSE_SHOT_RPM = 1800.0; // TODO: Tune close shot RPM
    public static final double CLOSE_SHOT_HOOD = 0.0;    // TODO: Tune close shot hood pose

    /** Far shot velocity (RPM) */
    public static final double FAR_SHOT_RPM = 2000.0;                  // TODO: Tune far shot RPM
    public static final double FAR_SHOT_HOOD = MAX_HOOD_POSE_ROT * 0.5; // TODO: Tune far shot hood pose

    /** Pass shot presets */
    public static final double PASS_SHOT_RPM = 3000.0;                                     // TODO: Tune pass RPM
    public static final double PASS_SHOT_HOOD = MAX_HOOD_POSE_ROT - (0.10 * MAX_HOOD_POSE_ROT); // TODO: Tune

    /**
     * SPINUP: ~80% of lowest real shot RPM for quick ramp to target.
     * Example: if your lowest shot is ~1800 RPM, this gives ~1440 RPM hold.
     */
    private static final double SPINUP_RPM = 0.80 * CLOSE_SHOT_RPM;

    /** Default flywheel velocity testing increment (RPM) */
    public static final double FLYWHEEL_TEST_INCREMENT_RPM = 100.0;

    /** Default target RPM for flywheel ramp-up testing */
    public static final double RAMP_TEST_TARGET_RPM = 1800.0; // TODO Test this value

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

    /**
     * Creates a new ShooterSubsystem.
     *
     * @param io Hardware interface (real hardware or simulation)
     */
    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        // Initialize NetworkTables publishers for Elastic dashboard
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
    }

    // =========================================================================
    // PERIODIC
    // =========================================================================

    @Override
    public void periodic() {
        // Always: read control-critical CAN signals (flywheel velocity, voltage, hood position)
        io.updateInputs(inputs);

        // Always: run state machine logic
        updateStateMachine();

        // Every 5th cycle (10Hz): read diagnostic CAN signals + publish to Elastic
        if (++periodicCounter % 5 == 0) {
            io.updateSlowInputs(inputs);
            publishToElastic();
        }
    }

    /**
     * Publishes shooter telemetry to NetworkTables for Elastic dashboard.
     * Called at 10Hz — Elastic dashboard can't meaningfully display faster than this,
     * so publishing at 50Hz just wastes CPU on NetworkTables serialization.
     */
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

    // =========================================================================
    // STATE MACHINE
    // =========================================================================

    /**
     * Updates the state machine based on current state and sensor readings.
     *
     * State transitions:
     * - IDLE: Manual transition only (via spinup(), pass(), eject())
     * - SPINUP: Stays in SPINUP until transitioned to READY via prepareToShoot()
     * - READY: Stays in READY, IO layer works to recover if targets drift
     * - PASS: Stays in PASS, IO layer works to recover if targets drift
     * - EJECT: Manual transition out only (via setIdle())
     */
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                break; // manual transition only

            case SPINUP:
                break; // transition to READY happens via prepareToShoot()

            case READY:
                // IO layer keeps commanding targets — no state change needed
                break;

            case PASS:
                // IO layer keeps commanding targets — no state change needed
                break;

            case EJECT:
                break; // manual transition out only
        }
    }

    // ===== State Transitions =====

    /**
     * Transitions to a new state and executes entry actions.
     * Caches the state string to avoid per-cycle toString() allocation.
     */
    private void setState(ShooterState newState) {
        if (currentState == newState) {
            return; // Already in this state
        }

        Logger.recordOutput("Shooter/StateTransition",
                currentStateString + " -> " + newState.toString());

        currentState = newState;
        currentStateString = newState.toString(); // cache once per transition

        // Execute entry actions for new state
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
                // Targets set externally via setTargetVelocity/setTargetHoodPose
                // or via updateFromDistance() for vision-based shooting
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

    // =========================================================================
    // PUBLIC COMMAND METHODS (called by ShooterCommands or button bindings)
    // =========================================================================

    /** Sets the shooter to idle state (motors stopped, hood at MIN_POSE). */
    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    /**
     * Starts spinning up the flywheel at SPINUP_RPM.
     * This decreases time to hit target velocity when transitioning to READY.
     */
    public void spinup() {
        setState(ShooterState.SPINUP);
    }

    /**
     * Prepares shooter for shooting with current targets.
     * Call setTargetVelocity/setTargetHoodPose first, or use a preset.
     */
    public void prepareToShoot() {
        setState(ShooterState.READY);
    }

    /** Sets shooter to passing mode (50% max velocity, hood at MAX_POSE). */
    public void pass() {
        setState(ShooterState.PASS);
    }

    /** Starts ejecting (reverse flywheel) to clear jams. Hood moves to MIN_POSE. */
    public void eject() {
        setState(ShooterState.EJECT);
    }

    // ===== Shooting Presets =====

    /** Configures and activates close shot preset. */
    public void closeShot() {
        setTargetVelocity(CLOSE_SHOT_RPM);
        setTargetHoodPose(CLOSE_SHOT_HOOD);
        setState(ShooterState.READY);
    }

    /** Configures and activates far shot preset. */
    public void farShot() {
        setTargetVelocity(FAR_SHOT_RPM);
        setTargetHoodPose(FAR_SHOT_HOOD);
        setState(ShooterState.READY);
    }

    // ===== Target Setters =====

    /**
     * Sets the target flywheel velocity.
     * Does NOT change state — call prepareToShoot() after.
     */
    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), MAX_FLYWHEEL_MOTOR_RPM);
    }

    /**
     * Sets the target hood pose in rotations.
     * Clamped to valid range.
     */
    public void setTargetHoodPose(double rotations) {
        targetHoodPoseRot = Math.max(MIN_HOOD_POSE_ROT, Math.min(MAX_HOOD_POSE_ROT, rotations));
    }

    /**
     * Adjusts the target flywheel velocity by a delta.
     * Useful for manual bump-testing via controller buttons.
     */
    public void adjustTargetVelocity(double deltaRPM) {
        setTargetVelocity(targetFlywheelMotorRPM + deltaRPM);
    }

    /** Convenience: increase target velocity by FLYWHEEL_TEST_INCREMENT_RPM. */
    public void increaseTargetVelocity() {
        adjustTargetVelocity(FLYWHEEL_TEST_INCREMENT_RPM);
    }

    /** Convenience: decrease target velocity by FLYWHEEL_TEST_INCREMENT_RPM. */
    public void decreaseTargetVelocity() {
        adjustTargetVelocity(-FLYWHEEL_TEST_INCREMENT_RPM);
    }

    // =========================================================================
    // FLYWHEEL RAMP TESTING
    // =========================================================================

    /**
     * Creates a command that ramps the flywheel up to a target RPM from idle,
     * then returns to idle when the command ends.
     * The actual ramp rate is governed by ClosedLoopRamps in TalonFXConfigs.
     * Use this to test belt slip behavior during acceleration.
     *
     * @param targetRPM Target flywheel velocity
     * @return Command that ramps flywheel and idles on cancel
     */
    public Command flywheelRampTest(double targetRPM) {
        return Commands.startEnd(
                () -> {
                    setTargetVelocity(targetRPM);
                    prepareToShoot();
                },
                this::setIdle,
                this).withName("FlywheelRampTest");
    }

    // =========================================================================
    // VISION INTEGRATION
    // =========================================================================

    /**
     * Updates shooter targets based on distance to target.
     * Uses simple linear interpolation between close and far shots.
     * Does NOT change state — call prepareToShoot() after to transition to READY.
     *
     * @param distanceMeters Distance to target in meters
     */
    public void updateFromDistance(double distanceMeters) {
        // TODO: Replace with proper ballistic calculations
        // Clamp distance to reasonable range
        double distance = Math.max(1.0, Math.min(5.0, distanceMeters));

        // Linear interpolation: distance 1m → close shot, 5m → far shot
        double t = (distance - 1.0) / (5.0 - 1.0); // 0.0 to 1.0
        double velocity = CLOSE_SHOT_RPM + t * (FAR_SHOT_RPM - CLOSE_SHOT_RPM);
        double pose = CLOSE_SHOT_HOOD + t * (FAR_SHOT_HOOD - CLOSE_SHOT_HOOD);

        setTargetVelocity(velocity);
        setTargetHoodPose(pose);

        Logger.recordOutput("Shooter/VisionDistance", distanceMeters);
        Logger.recordOutput("Shooter/CalculatedRPM", velocity);
        Logger.recordOutput("Shooter/CalculatedPose", pose);
    }

    // =========================================================================
    // STATUS QUERIES
    // =========================================================================

    /**
     * Returns true if shooter is ready to shoot
     * (in READY state with both flywheel and hood at targets).
     */
    public boolean isReady() {
        return currentState == ShooterState.READY && isFlywheelAtVelocity() && isHoodAtPose();
    }

    /**
     * Returns true if shooter is in PASS state with both flywheel and hood at targets.
     */
    public boolean isPassReady() {
        return currentState == ShooterState.PASS && isFlywheelAtVelocity() && isHoodAtPose();
    }

    /**
     * Returns true if flywheel is at target velocity (within percentage tolerance).
     */
    public boolean isFlywheelAtVelocity() {
        if (Math.abs(targetFlywheelMotorRPM) < 1.0) {
            return Math.abs(inputs.flywheelLeaderMotorRPM) < 50.0; // within 50 RPM of stopped
        }
        double tolerance = Math.abs(targetFlywheelMotorRPM) * FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    /**
     * Returns true if hood is at target pose (within HOOD_POSE_TOLERANCE rotations).
     */
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

    /** Gets current state. */
    public ShooterState getState() {
        return currentState;
    }

    /** Gets current flywheel velocity in RPM. */
    public double getCurrentVelocityRPM() {
        return inputs.flywheelLeaderMotorRPM;
    }

    /** Gets current hood pose in rotations. */
    public double getCurrentHoodPose() {
        return inputs.hoodPositionRotations;
    }

    /** Gets target flywheel velocity in RPM. */
    public double getTargetVelocityRPM() {
        return targetFlywheelMotorRPM;
    }

    /** Gets target hood pose in rotations. */
    public double getTargetHoodPose() {
        return targetHoodPoseRot;
    }

    // ===== Diagnostics =====

    /**
     * Returns true if total flywheel current is too high.
     */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0; // TODO: Tune threshold (sum of 3 motors)
    }
}