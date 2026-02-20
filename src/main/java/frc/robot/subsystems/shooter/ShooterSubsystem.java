package frc.robot.subsystems.shooter;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

/**
 * ShooterSubsystem - Shooter subsystem, command-based (no state machine).
 *
 * Commands control shooter behavior directly — no state machine in periodic().
 * This keeps the command scheduler fully in charge and avoids conflicts.
 *
 * USAGE:
 * 1. Use command factories (shoot3600, closeShot, etc.) in RobotContainer
 * 2. Use whileTrue() for held shots — flywheel stops automatically on release
 * 3. Use onTrue() + shootAndFeed() for full indexed shot sequences
 *
 * EXAMPLE BUTTON BINDING:
 *   operatorController.a().whileTrue(shooter.shoot3600());
 *   operatorController.b().onTrue(shooter.shootAndFeed(indexer));
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */
public class ShooterSubsystem extends SubsystemBase {

    // ===== Hardware Interface =====
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // ===== NetworkTables Publishers for Elastic Dashboard =====
    private final NetworkTable shooterTable;
    private final BooleanPublisher readyPublisher;
    private final DoublePublisher flywheelRpmPublisher;
    private final DoublePublisher targetRpmPublisher;
    private final DoublePublisher flywheelErrorPublisher;
    private final DoublePublisher hoodErrorPublisher;
    private final BooleanPublisher hoodAtPosePublisher;
    private final DoublePublisher flywheelVoltsPublisher;
    private final DoublePublisher throughBorePositionPublisher;
    private final BooleanPublisher throughBoreConnectedPublisher;

    // ===== Target Tracking =====
    // These track what we've most recently commanded, for telemetry and isReady() checks
    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot = MIN_HOOD_POSE_ROT;

    // ===== Periodic Cycle Counter =====
    // Throttle slow CAN reads and NT publishing to 10Hz (every 5th cycle)
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
     * 10% tolerance — forgiving for initial testing, tighten for competition
     */
    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.10;

    /** Hood pose tolerance (rotations) */
    private static final double HOOD_POSE_TOLERANCE = 0.10; // TODO Tune for shot consistency

    /** Testing increment for manual hood adjustment (rotations) */
    public static final double HOOD_TEST_INCREMENT = 0.5;

    // ===== Shooting Presets =====

    /** Close shot */
    public static final double CLOSE_SHOT_RPM  = 1800.0; // TODO Tune RPM
    public static final double CLOSE_SHOT_HOOD = 0.0;

    /** Far shot */
    public static final double FAR_SHOT_RPM  = 3000.0;                   // TODO Tune RPM
    public static final double FAR_SHOT_HOOD = MAX_HOOD_POSE_ROT * 0.5;  // TODO Tune hood

    /** Pass shot */
    public static final double PASS_SHOT_RPM  = 3000.0;                                      // TODO Tune RPM
    public static final double PASS_SHOT_HOOD = MAX_HOOD_POSE_ROT - (0.10 * MAX_HOOD_POSE_ROT); // TODO Tune hood
    
    /** Eject: reverse flywheel to clear jams */
    private static final double EJECT_VELOCITY_RPM = MAX_FLYWHEEL_MOTOR_RPM * -0.50;

    /** Testing increment */
    public static final double FLYWHEEL_TEST_INCREMENT_RPM = 100.0;

    /** Default target for ramp-up testing */
    public static final double RAMP_TEST_TARGET_RPM = 2800.0; // TODO Test

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

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        shooterTable = inst.getTable("Shooter");

        readyPublisher               = shooterTable.getBooleanTopic("IsReady").publish();
        flywheelRpmPublisher         = shooterTable.getDoubleTopic("FlywheelLeaderMotorRPM").publish();
        targetRpmPublisher           = shooterTable.getDoubleTopic("TargetFlywheelLeaderMotorRPM").publish();
        flywheelErrorPublisher       = shooterTable.getDoubleTopic("FlywheelError").publish();
        hoodErrorPublisher           = shooterTable.getDoubleTopic("HoodError").publish();
        hoodAtPosePublisher          = shooterTable.getBooleanTopic("HoodAtPose").publish();
        flywheelVoltsPublisher       = shooterTable.getDoubleTopic("FlywheelAppliedVolts").publish();
        throughBorePositionPublisher = shooterTable.getDoubleTopic("ThroughBorePosition").publish();
        throughBoreConnectedPublisher= shooterTable.getBooleanTopic("ThroughBoreConnected").publish();
    }

    // =========================================================================
    // PERIODIC — telemetry only, no control logic
    // =========================================================================

    @Override
    public void periodic() {
        // Always: read control-critical CAN signals
        io.updateInputs(inputs);

        // Every 5th cycle (10Hz): read diagnostics + publish to Elastic
        if (++periodicCounter % 5 == 0) {
            io.updateSlowInputs(inputs);
            publishToElastic();
        }
    }

    /**
     * Publishes shooter telemetry to NetworkTables for Elastic dashboard.
     * Called at 10Hz — publishing faster just wastes CPU.
     */
    private void publishToElastic() {
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
    // LOW-LEVEL MOTOR METHODS
    // Commands call these — nothing else should
    // =========================================================================

    /**
     * Commands the flywheel to a target RPM and hood to a target pose.
     * Updates internal targets for isReady() checks.
     */
    private void setFlywheelAndHood(double rpm, double hoodPoseRot) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), MAX_FLYWHEEL_MOTOR_RPM);
        targetHoodPoseRot = Math.max(MIN_HOOD_POSE_ROT, Math.min(MAX_HOOD_POSE_ROT, hoodPoseRot));
        io.setFlywheelVelocity(targetFlywheelMotorRPM);
        io.setHoodPose(targetHoodPoseRot);
    }

    /** Stops flywheel and returns hood to home. */
    private void stopAndHome() {
        targetFlywheelMotorRPM = 0.0;
        targetHoodPoseRot = MIN_HOOD_POSE_ROT;
        io.stopFlywheels();
        io.setHoodPose(MIN_HOOD_POSE_ROT);
    }

    // =========================================================================
    // COMMAND FACTORIES
    // Use these in RobotContainer button bindings
    // =========================================================================

    /**
     * Spins flywheel to 3600 RPM, hood at close shot pose.
     * Designed for whileTrue() — stops automatically when button is released.
     *
     * EXAMPLE: operatorController.a().whileTrue(shooter.shoot3600());
     */
    public Command shoot3603() {
        return Commands.startEnd(
            () -> setFlywheelAndHood(3603.0, CLOSE_SHOT_HOOD),
            this::stopAndHome,
            this
        ).withName("Shoot3600");
    }

    /**
     * Close shot preset (CLOSE_SHOT_RPM + CLOSE_SHOT_HOOD).
     * Designed for whileTrue() — stops automatically when button is released.
     */
    public Command closeShot() {
        return Commands.startEnd(
            () -> setFlywheelAndHood(CLOSE_SHOT_RPM, CLOSE_SHOT_HOOD),
            this::stopAndHome,
            this
        ).withName("CloseShot");
    }

    /**
     * Far shot preset (FAR_SHOT_RPM + FAR_SHOT_HOOD).
     * Designed for whileTrue() — stops automatically when button is released.
     */
    public Command farShot() {
        return Commands.startEnd(
            () -> setFlywheelAndHood(FAR_SHOT_RPM, FAR_SHOT_HOOD),
            this::stopAndHome,
            this
        ).withName("FarShot");
    }

    /**
     * Pass shot (PASS_SHOT_RPM + PASS_SHOT_HOOD).
     * Designed for whileTrue() — stops automatically when button is released.
     */
    public Command passShot() {
        return Commands.startEnd(
            () -> setFlywheelAndHood(PASS_SHOT_RPM, PASS_SHOT_HOOD),
            this::stopAndHome,
            this
        ).withName("PassShot");
    }

    /**
     * Eject — reverses flywheel to clear jams, hood at home.
     * Designed for whileTrue() — stops automatically when button is released.
     */
    public Command eject() {
        return Commands.startEnd(
            () -> {
                targetFlywheelMotorRPM = EJECT_VELOCITY_RPM;
                targetHoodPoseRot = MIN_HOOD_POSE_ROT;
                io.setFlywheelVelocity(EJECT_VELOCITY_RPM);
                io.setHoodPose(MIN_HOOD_POSE_ROT);
            },
            this::stopAndHome,
            this
        ).withName("Eject");
    }

    /**
     * Spin up at a custom RPM, stop and home on release.
     * Useful for testing or vision-calculated velocities.
     *
     * @param rpm Target flywheel velocity
     * @param hoodPoseRot Target hood pose in rotations
     */
    public Command shootAt(double rpm, double hoodPoseRot) {
        return Commands.startEnd(
            () -> setFlywheelAndHood(rpm, hoodPoseRot),
            this::stopAndHome,
            this
        ).withName("ShootAt" + (int) rpm + "RPM");
    }

    /**
     * Spin up, wait until ready, then run the indexer feed action.
     * Use this for a full indexed shot — call with onTrue().
     *
     * EXAMPLE:
     *   operatorController.a().onTrue(shooter.shootAndFeed(3600, CLOSE_SHOT_HOOD, indexer));
     *
     * @param rpm           Target flywheel RPM
     * @param hoodPoseRot   Target hood pose
     * @param feedAction    Command from your indexer to feed the game piece
     */
    public Command shootAndFeed(double rpm, double hoodPoseRot, Command feedAction) {
        return Commands.sequence(
            Commands.runOnce(() -> setFlywheelAndHood(rpm, hoodPoseRot), this),
            Commands.waitUntil(this::isReady).withTimeout(3.0),
            feedAction
        ).finallyDo(interrupted -> stopAndHome())
         .withName("ShootAndFeed");
    }

    /**
     * Flywheel ramp test — ramps to target RPM, stops and homes on cancel.
     * Governed by ClosedLoopRamps in TalonFXConfigs.
     *
     * @param targetRPM Target flywheel velocity
     */
    public Command flywheelRampTest(double targetRPM) {
        return Commands.startEnd(
            () -> setFlywheelAndHood(targetRPM, CLOSE_SHOT_HOOD),
            this::stopAndHome,
            this
        ).withName("FlywheelRampTest");
    }

    // ===== Manual Bump Adjustments =====
    // Useful for tuning sessions via controller buttons

    /** Increases target flywheel velocity by FLYWHEEL_TEST_INCREMENT_RPM and re-commands. */
    public Command increaseVelocity() {
        return Commands.runOnce(() -> {
            targetFlywheelMotorRPM = Math.min(targetFlywheelMotorRPM + FLYWHEEL_TEST_INCREMENT_RPM, MAX_FLYWHEEL_MOTOR_RPM);
            io.setFlywheelVelocity(targetFlywheelMotorRPM);
        }, this).withName("IncreaseVelocity");
    }

    /** Decreases target flywheel velocity by FLYWHEEL_TEST_INCREMENT_RPM and re-commands. */
    public Command decreaseVelocity() {
        return Commands.runOnce(() -> {
            targetFlywheelMotorRPM = Math.max(targetFlywheelMotorRPM - FLYWHEEL_TEST_INCREMENT_RPM, 0.0);
            io.setFlywheelVelocity(targetFlywheelMotorRPM);
        }, this).withName("DecreaseVelocity");
    }

    /** Increases hood pose by HOOD_TEST_INCREMENT and re-commands. */
    public Command increaseHood() {
        return Commands.runOnce(() -> {
            targetHoodPoseRot = Math.min(targetHoodPoseRot + HOOD_TEST_INCREMENT, MAX_HOOD_POSE_ROT);
            io.setHoodPose(targetHoodPoseRot);
        }, this).withName("IncreaseHood");
    }

    /** Decreases hood pose by HOOD_TEST_INCREMENT and re-commands. */
    public Command decreaseHood() {
        return Commands.runOnce(() -> {
            targetHoodPoseRot = Math.max(targetHoodPoseRot - HOOD_TEST_INCREMENT, MIN_HOOD_POSE_ROT);
            io.setHoodPose(targetHoodPoseRot);
        }, this).withName("DecreaseHood");
    }

    // =========================================================================
    // VISION INTEGRATION
    // =========================================================================

    /**
     * Updates shooter targets based on distance to target using linear interpolation.
     * Call this from a vision command before issuing a shoot command.
     *
     * @param distanceMeters Distance to target in meters
     */
    public void updateFromDistance(double distanceMeters) {
        double distance = Math.max(1.0, Math.min(5.0, distanceMeters));
        double t = (distance - 1.0) / (5.0 - 1.0);
        double velocity = CLOSE_SHOT_RPM + t * (FAR_SHOT_RPM - CLOSE_SHOT_RPM);
        double pose = CLOSE_SHOT_HOOD + t * (FAR_SHOT_HOOD - CLOSE_SHOT_HOOD);
        setFlywheelAndHood(velocity, pose);
    }

    // =========================================================================
    // STATUS QUERIES
    // =========================================================================

    /** Returns true if flywheel and hood are both at their targets. */
    public boolean isReady() {
        return isFlywheelAtVelocity() && isHoodAtPose();
    }

    /** Returns true if flywheel is within tolerance of target RPM. */
    public boolean isFlywheelAtVelocity() {
        if (Math.abs(targetFlywheelMotorRPM) < 1.0) {
            return Math.abs(inputs.flywheelLeaderMotorRPM) < 50.0;
        }
        double tolerance = Math.abs(targetFlywheelMotorRPM) * FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    /** Returns true if hood is within HOOD_POSE_TOLERANCE of target pose. */
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

    /** Returns true if flywheel current is too high (possible jam or mechanical issue). */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0; // TODO Tune threshold (sum of 3 motors)
    }
}