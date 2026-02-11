package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ShooterSubsystem - Main shooter subsystem with state machine control.
 *
 * STATE MACHINE:
 * - IDLE: All motors off, hood at home position
 * - SPINUP: Flywheel accelerating to target, hood moving to target angle
 * - READY: Both flywheel and hood at target, ready to shoot
 * - EJECT: Flywheel reverse for clearing jams
 *
 * USAGE:
 * 1. Call setTargetVelocity() and setTargetHoodAngle() to set desired shooting parameters
 * 2. Call spinup() to start motors
 * 3. Wait for isReady() to return true
 * 4. Signal indexer to feed game piece
 * 5. Call idle() when done
 *
 * INTEGRATION WITH VISION:
 * - Use updateFromDistance() to automatically calculate velocity/angle from target distance
 * - Vision subsystem provides distance -> shooter calculates ballistics
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */
public class ShooterSubsystem extends SubsystemBase {

    // ===== State Machine =====
    public enum ShooterState {
        IDLE,      // Flywheel stopped, hood at MIN_POSE
        SPINUP,    // Pre-rev flywheel (20% max), hood at mid position (0.5 * MAX_POSE)
        READY,     // Flywheel and hood at vision-based targets, ready to shoot
        PASS,      // Passing shot: 50% max velocity, hood at MAX_POSE
        EJECT      // Clearing jams: -50% max velocity, hood at MIN_POSE
    }

    // ===== Hardware Interface =====
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // ===== NetworkTables Publishers for Elastic Dashboard =====
    private final NetworkTable shooterTable;
    private final StringPublisher statePublisher;
    private final BooleanPublisher readyPublisher;
    private final DoublePublisher flywheelRpmPublisher;
    private final DoublePublisher targetRpmPublisher;
    private final DoublePublisher hoodAnglePublisher;
    private final DoublePublisher targetHoodPublisher;
    private final DoublePublisher flywheelErrorPublisher;
    private final DoublePublisher hoodErrorPublisher;

    // ===== NetworkTables Tuning Subscribers (read from Elastic) =====
    private final NetworkTable tuningTable;
    private final DoubleSubscriber tuneRpmSubscriber;
    private final DoubleSubscriber tuneAngleSubscriber;
    private final BooleanSubscriber tuneEnableSubscriber;

    // ===== Tuning Publishers (write defaults so widgets appear in Elastic) =====
    private final DoublePublisher tuneRpmPublisher;
    private final DoublePublisher tuneAnglePublisher;
    private final BooleanPublisher tuneEnablePublisher;
    private final BooleanPublisher tuneActivePublisher;

    // ===== State =====
    private ShooterState currentState = ShooterState.IDLE;
    private double targetFlywheelRPM = 0.0;
    private double targetHoodPose = MIN_HOOD_POSE;

    /** When true, tuning values from Elastic override normal targets */
    private boolean tuningActive = false;

    // ===== Constants =====
    /** Maximum flywheel velocity (RPM) */
    private static final double MAX_VELOCITY_RPM = 6000.0;  // TODO: Find actual max RPM

    /** Minimum hood pose (degrees) - used for IDLE and EJECT */
    private static final double MIN_HOOD_POSE = 15.0;  // TODO: Measure actual range

    /** Maximum hood pose (degrees) - used for PASS */
    private static final double MAX_HOOD_POSE = 60.0;  // TODO: Measure actual range

    /** Flywheel velocity tolerance (percentage of target, 0.0-1.0) */
    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.03;  // 3% tolerance — tight enough for accuracy, forgiving enough for real motors

    /** Hood pose tolerance (degrees) */
    private static final double HOOD_POSE_TOLERANCE = 0.5;  // 0.5 degree tolerance — critical for shot consistency

    // ===== State-Specific Constants =====
    /** SPINUP: Pre-rev flywheel velocity (20% of max) */
    private static final double SPINUP_VELOCITY_PERCENT = 0.20;
    private static final double SPINUP_VELOCITY_RPM = MAX_VELOCITY_RPM * SPINUP_VELOCITY_PERCENT;

    /** SPINUP: Hood pose (50% of max) */
    private static final double SPINUP_POSE_PERCENT = 0.50;
    private static final double SPINUP_HOOD_POSE = MIN_HOOD_POSE + (MAX_HOOD_POSE - MIN_HOOD_POSE) * SPINUP_POSE_PERCENT;

    /** PASS: Flywheel velocity (50% of max) */
    private static final double PASS_VELOCITY_PERCENT = 0.50;
    private static final double PASS_VELOCITY_RPM = MAX_VELOCITY_RPM * PASS_VELOCITY_PERCENT;

    /** EJECT: Flywheel velocity (-50% of max, negative for reverse) */
    private static final double EJECT_VELOCITY_PERCENT = -0.50;
    private static final double EJECT_VELOCITY_RPM = MAX_VELOCITY_RPM * EJECT_VELOCITY_PERCENT;

    // ===== Shooting Presets (for READY state) =====
    /** Close shot velocity (RPM) */
    public static final double CLOSE_SHOT_RPM = 3000.0;  // TODO: Tune
    public static final double CLOSE_SHOT_ANGLE = 25.0;  // TODO: Tune

    /** Far shot velocity (RPM) */
    public static final double FAR_SHOT_RPM = 5000.0;  // TODO: Tune
    public static final double FAR_SHOT_ANGLE = 45.0;  // TODO: Tune

    /** Hood adjustment increment (degrees) for manual testing */
    public static final double HOOD_ADJUST_INCREMENT = 5.0;

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
        flywheelRpmPublisher = shooterTable.getDoubleTopic("FlywheelRPM").publish();
        targetRpmPublisher = shooterTable.getDoubleTopic("TargetFlywheelRPM").publish();
        hoodAnglePublisher = shooterTable.getDoubleTopic("HoodAngle").publish();
        targetHoodPublisher = shooterTable.getDoubleTopic("TargetHoodAngle").publish();
        flywheelErrorPublisher = shooterTable.getDoubleTopic("FlywheelError").publish();
        hoodErrorPublisher = shooterTable.getDoubleTopic("HoodError").publish();

        // Initialize tuning table — separate subtable keeps Elastic organized
        // In Elastic, add Number Slider widgets bound to /Shooter/Tuning/RPM and /Shooter/Tuning/Angle
        // and a Toggle Switch bound to /Shooter/Tuning/Enable
        tuningTable = shooterTable.getSubTable("Tuning");

        // Publishers write default values so Elastic widgets auto-populate on first connect
        tuneRpmPublisher = tuningTable.getDoubleTopic("RPM").publish();
        tuneAnglePublisher = tuningTable.getDoubleTopic("Angle").publish();
        tuneEnablePublisher = tuningTable.getBooleanTopic("Enable").publish();
        tuneActivePublisher = tuningTable.getBooleanTopic("Active").publish();

        // Subscribers read values back when changed from the Elastic dashboard
        tuneRpmSubscriber = tuningTable.getDoubleTopic("RPM").subscribe(CLOSE_SHOT_RPM);
        tuneAngleSubscriber = tuningTable.getDoubleTopic("Angle").subscribe(CLOSE_SHOT_ANGLE);
        tuneEnableSubscriber = tuningTable.getBooleanTopic("Enable").subscribe(false);

        // Publish initial defaults so widgets appear in Elastic with sensible starting values
        tuneRpmPublisher.set(CLOSE_SHOT_RPM);
        tuneAnglePublisher.set(CLOSE_SHOT_ANGLE);
        tuneEnablePublisher.set(false);
        tuneActivePublisher.set(false);
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // ===== Live Tuning from Elastic Dashboard =====
        // When "Enable" toggle is switched on in Elastic, tuning values override normal targets.
        // This lets students adjust RPM and angle from the dashboard without redeploying code.
        boolean tuneEnabled = tuneEnableSubscriber.get();
        if (tuneEnabled) {
            double tuneRPM = tuneRpmSubscriber.get();
            double tuneAngle = tuneAngleSubscriber.get();

            // Apply tuning values — overrides preset/vision targets
            setTargetVelocity(tuneRPM);
            setTargetHoodPose(tuneAngle);

            // Auto-transition to READY so the tuning values actually run the motors
            if (currentState != ShooterState.READY) {
                prepareToShoot();
            }

            tuningActive = true;
        } else if (tuningActive) {
            // Tuning was just disabled — return to idle for safety
            setIdle();
            tuningActive = false;
        }

        // Publish tuning active state back to Elastic (for status indicator)
        tuneActivePublisher.set(tuningActive);

        // Update state machine
        updateStateMachine();

        // Log state and targets
        Logger.recordOutput("Shooter/State", currentState.toString());
        Logger.recordOutput("Shooter/TargetFlywheelRPM", targetFlywheelRPM);
        Logger.recordOutput("Shooter/TargetHoodPose", targetHoodPose);
        Logger.recordOutput("Shooter/IsReady", isReady());
        Logger.recordOutput("Shooter/IsPassReady", isPassReady());
        Logger.recordOutput("Shooter/FlywheelAtVelocity", isFlywheelAtVelocity());
        Logger.recordOutput("Shooter/HoodAtPose", isHoodAtPose());
        Logger.recordOutput("Shooter/FlywheelError", getFlywheelError());
        Logger.recordOutput("Shooter/HoodError", getHoodError());
        Logger.recordOutput("Shooter/TuningActive", tuningActive);

        // WCP ThroughBore Encoder (secondary feedback)
        Logger.recordOutput("Shooter/ThroughBore/PositionRotations", inputs.hoodThroughBorePositionRotations);
        Logger.recordOutput("Shooter/ThroughBore/PositionDegrees", inputs.hoodThroughBorePositionDegrees);
        Logger.recordOutput("Shooter/ThroughBore/Connected", inputs.hoodThroughBoreConnected);

        // Publish to NetworkTables for Elastic dashboard
        statePublisher.set(currentState.toString());
        readyPublisher.set(isReady());
        flywheelRpmPublisher.set(inputs.flywheelVelocityRPM);
        targetRpmPublisher.set(targetFlywheelRPM);
        hoodAnglePublisher.set(inputs.hoodAngleDegrees);
        targetHoodPublisher.set(targetHoodPose);
        flywheelErrorPublisher.set(getFlywheelError());
        hoodErrorPublisher.set(getHoodError());
    }

    /**
     * Updates the state machine based on current state and sensor readings.
     *
     * State transitions:
     * - IDLE: Manual transition only (via spinup(), pass(), eject())
     * - SPINUP: Stays in SPINUP until transitioned to READY via prepareToShoot()
     * - READY: Falls back to SPINUP if targets are lost
     * - PASS: Falls back to re-acquiring if targets are lost
     * - EJECT: Manual transition out only (via setIdle())
     */
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                // Stay idle - manual transition only
                break;

            case SPINUP:
                // Stay in SPINUP - this is a pre-rev state
                // Transition to READY happens via prepareToShoot() when vision targets are set
                break;

            case READY:
                // Check if we've lost velocity/position (defensive)
                if (!isFlywheelAtVelocity() || !isHoodAtPose()) {
                    // Lost targets - but stay in READY and keep commanding targets
                    // The IO layer will work to recover
                }
                break;

            case PASS:
                // Check if we've lost velocity/position (defensive)
                if (!isFlywheelAtVelocity() || !isHoodAtPose()) {
                    // Lost targets - but stay in PASS and keep commanding targets
                }
                break;

            case EJECT:
                // Stay ejecting - manual transition out only
                break;
        }
    }

    // ===== State Transitions =====

    /**
     * Transitions to a new state and executes entry actions.
     */
    private void setState(ShooterState newState) {
        if (currentState == newState) {
            return;  // Already in this state
        }

        // Log state transition
        Logger.recordOutput("Shooter/StateTransition",
            currentState.toString() + " -> " + newState.toString());

        currentState = newState;

        // Execute entry actions for new state
        switch (newState) {
            case IDLE:
                // Stop flywheels, hood to MIN_POSE
                io.stopFlywheels();
                targetFlywheelRPM = 0.0;
                targetHoodPose = MIN_HOOD_POSE;
                io.setHoodPose(MIN_HOOD_POSE);
                break;

            case SPINUP:
                // Pre-rev: 20% max velocity, hood at mid position
                targetFlywheelRPM = SPINUP_VELOCITY_RPM;
                targetHoodPose = SPINUP_HOOD_POSE;
                io.setFlywheelVelocity(SPINUP_VELOCITY_RPM);
                io.setHoodPose(SPINUP_HOOD_POSE);
                break;

            case READY:
                // Targets set externally via setTargetVelocity/setTargetHoodPose
                // or via updateFromDistance() for vision-based shooting
                io.setFlywheelVelocity(targetFlywheelRPM);
                io.setHoodPose(targetHoodPose);
                break;

            case PASS:
                // Passing: 50% max velocity, hood at MAX_POSE
                targetFlywheelRPM = PASS_VELOCITY_RPM;
                targetHoodPose = MAX_HOOD_POSE;
                io.setFlywheelVelocity(PASS_VELOCITY_RPM);
                io.setHoodPose(MAX_HOOD_POSE);
                break;

            case EJECT:
                // Clearing jams: -50% max velocity (reverse), hood at MIN_POSE
                targetFlywheelRPM = EJECT_VELOCITY_RPM;
                targetHoodPose = MIN_HOOD_POSE;
                io.setFlywheelVelocity(EJECT_VELOCITY_RPM);
                io.setHoodPose(MIN_HOOD_POSE);
                break;
        }
    }

    // ===== Public Command Methods =====

    /**
     * Sets the shooter to idle state (motors stopped, hood at MIN_POSE).
     */
    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    /**
     * Starts spinning up to pre-rev the flywheel (20% max) and position hood (mid position).
     * This decreases time to hit target velocity when transitioning to READY.
     */
    public void spinup() {
        setState(ShooterState.SPINUP);
    }

    /**
     * Prepares shooter for shooting with specific targets.
     * Call setTargetVelocity/setTargetHoodPose first to set targets,
     * or use updateFromDistance() for vision-based targeting.
     */
    public void prepareToShoot() {
        setState(ShooterState.READY);
    }

    /**
     * Sets shooter to passing mode (50% max velocity, hood at MAX_POSE).
     */
    public void pass() {
        setState(ShooterState.PASS);
    }

    /**
     * Starts ejecting (reverse flywheel at -50% max) to clear jams.
     * Hood moves to MIN_POSE.
     */
    public void eject() {
        setState(ShooterState.EJECT);
    }

    /**
     * Stops ejecting and returns to idle.
     */
    public void stopEject() {
        setState(ShooterState.IDLE);
    }

    // ===== Target Setters (for READY state) =====

    /**
     * Sets the target flywheel velocity for READY state.
     * Call prepareToShoot() after setting targets to transition to READY.
     *
     * @param rpm Target velocity in RPM
     */
    public void setTargetVelocity(double rpm) {
        this.targetFlywheelRPM = Math.max(0, Math.min(MAX_VELOCITY_RPM, rpm));  // Clamp to valid range

        // If in READY state, update velocity immediately
        if (currentState == ShooterState.READY) {
            io.setFlywheelVelocity(targetFlywheelRPM);
        }
    }

    /**
     * Sets the target hood pose for READY state.
     * Call prepareToShoot() after setting targets to transition to READY.
     *
     * @param degrees Target pose in degrees
     */
    public void setTargetHoodPose(double degrees) {
        this.targetHoodPose = Math.max(MIN_HOOD_POSE,
                                       Math.min(MAX_HOOD_POSE, degrees));  // Clamp to range

        // If in READY state, update pose immediately
        if (currentState == ShooterState.READY) {
            io.setHoodPose(targetHoodPose);
        }
    }

    /**
     * Adjusts the target hood pose by the given delta (positive = increase, negative = decrease).
     * Clamps result to [MIN_HOOD_POSE, MAX_HOOD_POSE].
     * If in READY state, the new pose is applied immediately.
     *
     * @param deltaDegrees Amount to adjust hood pose in degrees
     */
    public void adjustHoodPose(double deltaDegrees) {
        setTargetHoodPose(targetHoodPose + deltaDegrees);
    }

    /**
     * Sets counter-wheel velocity for backspin.
     * Independent of state machine.
     *
     * @param rpm Counter-wheel velocity in RPM
     */
    public void setCounterWheelVelocity(double rpm) {
        io.setCounterWheelVelocity(rpm);
    }

    // ===== Preset Shots (convenience methods for READY state) =====

    /**
     * Configures shooter for a close shot and transitions to READY.
     */
    public void closeShot() {
        setTargetVelocity(CLOSE_SHOT_RPM);
        setTargetHoodPose(CLOSE_SHOT_ANGLE);
        prepareToShoot();
    }

    /**
     * Configures shooter for a far shot and transitions to READY.
     */
    public void farShot() {
        setTargetVelocity(FAR_SHOT_RPM);
        setTargetHoodPose(FAR_SHOT_ANGLE);
        prepareToShoot();
    }

    // ===== Vision Integration =====

    /**
     * Updates shooter targets based on distance to target.
     * Uses ballistic calculations to determine velocity and pose.
     * Does NOT change state - call prepareToShoot() after to transition to READY.
     *
     * @param distanceMeters Distance to target in meters
     */
    public void updateFromDistance(double distanceMeters) {
        // TODO: Implement ballistic calculations
        // For now, use simple linear interpolation between close and far shots

        // Clamp distance to reasonable range
        double distance = Math.max(1.0, Math.min(5.0, distanceMeters));

        // Linear interpolation: distance 1m → close shot, 5m → far shot
        double t = (distance - 1.0) / (5.0 - 1.0);  // 0.0 to 1.0

        double velocity = CLOSE_SHOT_RPM + t * (FAR_SHOT_RPM - CLOSE_SHOT_RPM);
        double pose = CLOSE_SHOT_ANGLE + t * (FAR_SHOT_ANGLE - CLOSE_SHOT_ANGLE);

        setTargetVelocity(velocity);
        setTargetHoodPose(pose);

        Logger.recordOutput("Shooter/VisionDistance", distanceMeters);
        Logger.recordOutput("Shooter/CalculatedRPM", velocity);
        Logger.recordOutput("Shooter/CalculatedPose", pose);
    }

    // ===== Status Queries =====

    /**
     * Returns true if shooter is ready to shoot (in READY state with both flywheel and hood at targets).
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
     * Uses FLYWHEEL_TOLERANCE_PERCENT of the target velocity.
     */
    public boolean isFlywheelAtVelocity() {
        // Handle zero target (IDLE state) - check if velocity is near zero
        if (Math.abs(targetFlywheelRPM) < 1.0) {
            return Math.abs(inputs.flywheelVelocityRPM) < 50.0;  // Within 50 RPM of stopped
        }
        // Percentage-based tolerance
        double tolerance = Math.abs(targetFlywheelRPM) * FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelVelocityRPM - targetFlywheelRPM) < tolerance;
    }

    /**
     * Returns true if hood is at target pose (within HOOD_POSE_TOLERANCE degrees).
     */
    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodAngleDegrees - targetHoodPose) < HOOD_POSE_TOLERANCE;
    }

    /**
     * Gets current flywheel velocity error (target - actual).
     */
    public double getFlywheelError() {
        return targetFlywheelRPM - inputs.flywheelVelocityRPM;
    }

    /**
     * Gets current hood pose error (target - actual).
     */
    public double getHoodError() {
        return targetHoodPose - inputs.hoodAngleDegrees;
    }

    /**
     * Gets current state.
     */
    public ShooterState getState() {
        return currentState;
    }

    /**
     * Gets current flywheel velocity.
     */
    public double getCurrentVelocityRPM() {
        return inputs.flywheelVelocityRPM;
    }

    /**
     * Gets current hood pose.
     */
    public double getCurrentHoodPose() {
        return inputs.hoodAngleDegrees;
    }

    /**
     * Gets target flywheel velocity.
     */
    public double getTargetVelocityRPM() {
        return targetFlywheelRPM;
    }

    /**
     * Gets target hood pose.
     */
    public double getTargetHoodPose() {
        return targetHoodPose;
    }

    // ===== Diagnostics =====

    /**
     * Returns true if any flywheel motor is over-temperature.
     */
    public boolean isOverheating() {
        return inputs.flywheelTempCelsius > 80.0;  // TODO: Tune threshold
    }

    /**
     * Returns true if total flywheel current is too high.
     */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0;  // TODO: Tune threshold (sum of 3 motors)
    }

    /**
     * Gets maximum difference between individual flywheel velocities.
     * Useful for detecting if followers are out of sync.
     */
    public double getFlywheelVelocitySpread() {
        double max = Math.max(Math.max(inputs.flywheelAVelocityRPM,
                                       inputs.flywheelBVelocityRPM),
                                       inputs.flywheelCVelocityRPM);
        double min = Math.min(Math.min(inputs.flywheelAVelocityRPM,
                                       inputs.flywheelBVelocityRPM),
                                       inputs.flywheelCVelocityRPM);
        return max - min;
    }
}
