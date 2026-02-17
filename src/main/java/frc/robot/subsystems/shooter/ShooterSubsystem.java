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
 * - EJECT: Flywheel reverse for clearing jams
 *
 * USAGE:
 * 1. Call setTargetVelocity() and setTargetHoodAngle() to set desired shooting
 * parameters
 * 2. Call spinup() to start motors
 * 3. Wait for isReady() to return true
 * 4. Signal indexer to feed game piece
 * 5. Call idle() when done
 *
 * INTEGRATION WITH VISION:
 * - Use updateFromDistance() to automatically calculate velocity/angle from
 * target distance
 * - Vision subsystem provides distance -> shooter calculates ballistics
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */
public class ShooterSubsystem extends SubsystemBase {

    // ===== State Machine =====
    public enum ShooterState {
        IDLE, // Flywheel stopped, hood at MIN_POSE
        SPINUP, // Pre-rev flywheel (20% max), hood at mid position (0.5 * MAX_POSE)
        READY, // Flywheel and hood at vision-based targets, ready to shoot
        PASS, // Passing shot: 50% max velocity, hood at MAX_POSE
        EJECT // Clearing jams: -50% max velocity, hood at MIN_POSE
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
    // private final DoublePublisher hoodAnglePublisher;
    // private final DoublePublisher targetHoodPublisher;
    private final DoublePublisher flywheelErrorPublisher;
    private final DoublePublisher hoodErrorPublisher;
    private final BooleanPublisher hoodAtPosePublisher;
    private final DoublePublisher flywheelVoltsPublisher;
    private final DoublePublisher flywheelMotorRpsPublisher;
    private final DoublePublisher throughBorePositionPublisher;
    private final BooleanPublisher throughBoreConnectedPublisher;

    // ===== State =====
    private ShooterState currentState = ShooterState.IDLE;
    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot = MIN_HOOD_POSE_ROT;

    // Tuning system disabled — uncomment when ready to re-enable Elastic dashboard
    // tuning
    // private boolean tuningActive = false;

    // ===== Constants =====
    /** Maximum flywheel velocity (RPM) Free Spin */
    private static final double MAX_FLYWHEEL_MOTOR_RPM = 6380.0;

    /** Minimum hood pose (rotations) */
    private static final double MIN_HOOD_POSE_ROT = 0.0;

    /** Maximum hood pose (rotations) */
    private static final double MAX_HOOD_POSE_ROT = 9.14;

    /**
     * Flywheel velocity tolerance (percentage of target, 0.0 to 1.0)
     * 
     * TODO Lower this as needed for more precise control
     * 
     * 3% tolerance — tight enough for accuracy, forgiving enough for real motors
     */
    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.10;

    /** Hood pose tolerance (rotations) */
    private static final double HOOD_POSE_TOLERANCE = 0.10; // TODO Set an appropriate 0.5 degree tolerance — critical
                                                            // for shot consistency

    /** Testing increment for manual hood adjustment (rotations). */
    public static final double HOOD_TEST_INCREMENT = 0.5; // 0.5 degree increments for fine-tuning hood position during
                                                          // testing

    // ===== State-Specific Constants =====
    /** SPINUP: Pre-rev flywheel velocity (20% of max) */
    private static final double SPINUP_VELOCITY_PERCENT = 0.20;

    private static final double SPINUP_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * SPINUP_VELOCITY_PERCENT;

    /** PASS: Flywheel velocity (50% of max) */
    private static final double PASS_VELOCITY_PERCENT = 0.50;
    private static final double PASS_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * PASS_VELOCITY_PERCENT;

    /** EJECT: Flywheel velocity (-50% of max, negative for reverse) */
    private static final double EJECT_VELOCITY_PERCENT = -0.50;
    private static final double EJECT_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * EJECT_VELOCITY_PERCENT;

    // ===== Shooting Presets (for READY state) =====
    /** Close shot velocity (RPM) */
    public static final double CLOSE_SHOT_RPM = 1000.0; // TODO: Tune close shot RPM
    public static final double CLOSE_SHOT_HOOD = 0.0; // TODO: Tune close shot hood pose
    // public static final double CLOSE_SHOT_ANGLE = 25.0;

    /** Far shot velocity (RPM) */
    public static final double FAR_SHOT_RPM = 1500.0; // TODO: Tune far shot RPM
    
    public static final double FAR_SHOT_HOOD = MAX_HOOD_POSE_ROT * (0.5); // TODO: Tune far shot hood pose
    
    // public static final double FAR_SHOT_ANGLE = 45.0; //

    public static final double PASS_SHOT_RPM = 1500.0; // TODO: Tune pass RPM

    // TODO: Tune pass shot Hood Pose
    public static final double PASS_SHOT_HOOD = MAX_HOOD_POSE_ROT - (0.10 * MAX_HOOD_POSE_ROT); 

    /** Default flywheel velocity testing increment (RPM) */
    public static final double FLYWHEEL_TEST_INCREMENT_RPM = 100.0;

    /** Default target RPM for flywheel ramp-up testing */
    public static final double RAMP_TEST_TARGET_RPM = 3000.0; // TODO Test this value // 1000
    // 1000 RPM is a soft lob
    //

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
        flywheelMotorRpsPublisher = shooterTable.getDoubleTopic("FlywheelLeaderMotorRPS").publish();
        throughBorePositionPublisher = shooterTable.getDoubleTopic("ThroughBorePosition").publish();
        throughBoreConnectedPublisher = shooterTable.getBooleanTopic("ThroughBoreConnected").publish();
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);
        // Logger.processInputs("Shooter", inputs); // FIXME Testing CPU without

        // Update state machine
        updateStateMachine();

            // Logger.processInputs("Shooter", inputs);

            // Logger.recordOutput("Shooter/State", currentState.toString());
            // Logger.recordOutput("Shooter/TargetFlywheelLeaderMotorRPM", targetFlywheelMotorRPM);
            // Logger.recordOutput("Shooter/TargetHoodPose", targetHoodPoseRot);
            // Logger.recordOutput("Shooter/IsReady", isReady());
            // Logger.recordOutput("Shooter/FlywheelError", getFlywheelError());
            // Logger.recordOutput("Shooter/HoodError", getHoodError());
            // Logger.recordOutput("Shooter/ThroughBore/PositionRotations", inputs.hoodThroughBorePositionRotations);
            // Logger.recordOutput("Shooter/ThroughBore/Connected", inputs.hoodThroughBoreConnected);

        // Keep Elastic NT publishing
        publishToElastic();

    }

    private void publishToElastic() {
        statePublisher.set(currentState.toString());
        readyPublisher.set(isReady());
        flywheelRpmPublisher.set(inputs.flywheelLeaderMotorRPM);
        targetRpmPublisher.set(targetFlywheelMotorRPM);
        flywheelErrorPublisher.set(getFlywheelError());
        hoodErrorPublisher.set(getHoodError());
        hoodAtPosePublisher.set(isHoodAtPose());
        flywheelVoltsPublisher.set(inputs.flywheelAppliedVolts);
        flywheelMotorRpsPublisher.set(inputs.flywheelMotorRPS);
        throughBorePositionPublisher.set(inputs.hoodThroughBorePositionRotations);
        throughBoreConnectedPublisher.set(inputs.hoodThroughBoreConnected);
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
            return; // Already in this state
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
                targetFlywheelMotorRPM = 0.0;
                targetHoodPoseRot = MIN_HOOD_POSE_ROT;
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case SPINUP:
                // Pre-rev: 20% max velocity, hood at mid position
                targetFlywheelMotorRPM = SPINUP_VELOCITY_RPM;
                io.setFlywheelVelocity(SPINUP_VELOCITY_RPM);
                break;

            case READY:
                // Targets set externally via setTargetVelocity/setTargetHoodPose
                // or via updateFromDistance() for vision-based shooting
                io.setFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;

            case PASS:
                // Passing: 50% max velocity, hood at MAX_POSE
                targetFlywheelMotorRPM = PASS_VELOCITY_RPM;
                targetHoodPoseRot = MAX_HOOD_POSE_ROT;
                io.setFlywheelVelocity(PASS_VELOCITY_RPM);
                io.setHoodPose(MAX_HOOD_POSE_ROT);
                break;

            case EJECT:
                // Clearing jams: -50% max velocity (reverse), hood at MIN_POSE
                targetFlywheelMotorRPM = EJECT_VELOCITY_RPM;
                targetHoodPoseRot = MIN_HOOD_POSE_ROT;
                io.setFlywheelVelocity(EJECT_VELOCITY_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
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
     * Starts spinning up to pre-rev the flywheel (20% max) and position hood (mid
     * position).
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
        this.targetFlywheelMotorRPM = Math.max(0, Math.min(MAX_FLYWHEEL_MOTOR_RPM, rpm)); // Clamp to valid range

        // If in READY state, update velocity immediately
        if (currentState == ShooterState.READY) {
            io.setFlywheelVelocity(targetFlywheelMotorRPM);
        }
    }

    /**
     * Sets the target hood pose for READY state.
     * Call prepareToShoot() after setting targets to transition to READY.
     *
     * @param degrees Target pose in degrees
     */
    public void setTargetHoodPose(double degrees) {
        this.targetHoodPoseRot = Math.max(MIN_HOOD_POSE_ROT,
                Math.min(MAX_HOOD_POSE_ROT, degrees)); // Clamp to range

        // If in READY state, update pose immediately
        if (currentState == ShooterState.READY) {
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    /**
     * Adjusts the hood target by a delta amount for manual testing.
     * Positive values raise the hood, negative values lower it.
     *
     * @param deltaDegrees Amount to add to current target hood pose
     */
    public void adjustTargetHoodPose(double deltaDegrees) {
        setTargetHoodPose(targetHoodPoseRot + deltaDegrees);
    }

    /** Raises hood target by the standard testing increment. */
    public void increaseHoodForTesting() {
        adjustTargetHoodPose(HOOD_TEST_INCREMENT);
    }

    /** Lowers hood target by the standard testing increment. */
    public void decreaseHoodForTesting() {
        adjustTargetHoodPose(-HOOD_TEST_INCREMENT);
    }

    /**
     * Commands the hood to MAX_HOOD_POSE_ROT using closed-loop position control.
     * Uses the existing PositionVoltage PID — precise and repeatable.
     */
    public Command runHoodToMax() {
        return Commands.runOnce(() -> {
            setTargetHoodPose(MAX_HOOD_POSE_ROT);
            if (currentState != ShooterState.READY) {
                prepareToShoot();
            }
        }, this);
    }

    /**
     * Commands the hood to MIN_HOOD_POSE_ROT using closed-loop position control.
     * Uses the existing PositionVoltage PID — precise and repeatable.
     */
    public Command runHoodToMin() {
        return Commands.runOnce(() -> {
            setTargetHoodPose(MIN_HOOD_POSE_ROT);
            if (currentState != ShooterState.READY) {
                prepareToShoot();
            }
        }, this);
    }

    // ===== Preset Shots (convenience methods for READY state) =====

    /**
     * Configures shooter for a close shot and transitions to READY.
     */
    public void closeShot() {
        // setTargetVelocity(CLOSE_SHOT_RPM);
        setTargetHoodPose(CLOSE_SHOT_HOOD);
        prepareToShoot();
    }

    /**
     * Configures shooter for a far shot and transitions to READY.
     */
    public void farShot() {
        // setTargetVelocity(FAR_SHOT_RPM);
        setTargetHoodPose(FAR_SHOT_HOOD);
        prepareToShoot();
    }

    // ===== Flywheel Velocity Adjustment (for testing) =====

    /**
     * Adjusts target flywheel velocity by a delta (positive increases, negative
     * decreases).
     * Transitions to READY state so the motors run at the new velocity.
     * Clamped to [0, MAX_FLYWHEEL_MOTOR_RPM] via setTargetVelocity().
     *
     * @param deltaRPM Amount to add to current target RPM
     */
    public void adjustTargetVelocity(double deltaRPM) {
        setTargetVelocity(targetFlywheelMotorRPM + deltaRPM);
        if (currentState != ShooterState.READY) {
            prepareToShoot();
        }
    }

    /**
     * Increases target flywheel velocity by the default testing increment.
     */
    public void increaseTargetVelocity() {
        adjustTargetVelocity(FLYWHEEL_TEST_INCREMENT_RPM);
    }

    /**
     * Decreases target flywheel velocity by the default testing increment.
     */
    public void decreaseTargetVelocity() {
        adjustTargetVelocity(-FLYWHEEL_TEST_INCREMENT_RPM);
    }

    // ===== Flywheel Ramp Testing =====

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
        double t = (distance - 1.0) / (5.0 - 1.0); // 0.0 to 1.0

        double velocity = CLOSE_SHOT_RPM + t * (FAR_SHOT_RPM - CLOSE_SHOT_RPM);
        double pose = CLOSE_SHOT_HOOD + t * (FAR_SHOT_HOOD - CLOSE_SHOT_HOOD);

        setTargetVelocity(velocity);
        setTargetHoodPose(pose);

        Logger.recordOutput("Shooter/VisionDistance", distanceMeters);
        Logger.recordOutput("Shooter/CalculatedRPM", velocity);
        Logger.recordOutput("Shooter/CalculatedPose", pose);
    }

    // ===== Status Queries =====

    /**
     * Returns true if shooter is ready to shoot (in READY state with both flywheel
     * and hood at targets).
     */
    public boolean isReady() {
        return currentState == ShooterState.READY && isFlywheelAtVelocity() && isHoodAtPose();
    }

    /**
     * Returns true if shooter is in PASS state with both flywheel and hood at
     * targets.
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
        if (Math.abs(targetFlywheelMotorRPM) < 1.0) {
            return Math.abs(inputs.flywheelLeaderMotorRPM) < 50.0; // Within 50 RPM of stopped
        }
        // Percentage-based tolerance
        double tolerance = Math.abs(targetFlywheelMotorRPM) * FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    /**
     * Returns true if hood is at target pose (within HOOD_POSE_TOLERANCE degrees).
     */
    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodPositionRotations - targetHoodPoseRot) < HOOD_POSE_TOLERANCE;
    }

    /**
     * Gets current flywheel velocity error (target - actual).
     */
    public double getFlywheelError() {
        return targetFlywheelMotorRPM - inputs.flywheelLeaderMotorRPM;
    }

    /**
     * Gets current hood pose error (target - actual).
     */
    public double getHoodError() {
        return targetHoodPoseRot - inputs.hoodPositionRotations;
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
        return inputs.flywheelLeaderMotorRPM;
    }

    /**
     * Gets current hood pose.
     */
    public double getCurrentHoodPose() {
        return inputs.hoodPositionRotations;
    }

    /**
     * Gets target flywheel velocity.
     */
    public double getTargetVelocityRPM() {
        return targetFlywheelMotorRPM;
    }

    /**
     * Gets target hood pose.
     */
    public double getTargetHoodPose() {
        return targetHoodPoseRot;
    }

    // ===== Diagnostics =====

    /**
     * Returns true if any flywheel motor is over-temperature.
     */
    // public boolean isOverheating() {
    // return inputs.flywheelTempCelsius > 80.0; // TODO: Tune threshold
    // }

    /**
     * Returns true if total flywheel current is too high.
     */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0; // TODO: Tune threshold (sum of 3 motors)
    }

}