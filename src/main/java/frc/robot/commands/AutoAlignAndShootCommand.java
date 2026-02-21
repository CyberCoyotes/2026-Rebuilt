package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Set;

/**
 * AutoAlignAndShootCommand - Used in autonomous to align to hub and shoot.
 * Reuses the same alignment, hood, and RPM interpolation logic as FarShotCommand,
 * but with no driver input and a configurable shot duration finish condition.
 */
public class AutoAlignAndShootCommand extends Command {

    // ===== Hub Tags =====
    private static final Set<Integer> HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    // ===== Distance Interpolation Range =====
    private static final double CLOSE_DISTANCE_METERS = 0.85;
    private static final double FAR_DISTANCE_METERS = 3.5;

    // ===== RPM Interpolation =====
    private static final double MIN_RPM = 2875;
    private static final double MAX_RPM = 3875;

    // ===== tx EMA Filter =====
    private static final double TX_FILTER_ALPHA = 0.2;

    // ===== Rotation PID =====
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.001;
    private static final double MIN_ROTATION_OUTPUT = 1.25;
    private static final double MAX_ROTATION_RATE = 4.0;
    private static final double ALIGN_TOLERANCE_DEGREES = 1.5;

    // ===== Hardware =====
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;

    // ===== Config =====
    private final double shootDurationSeconds;

    // ===== Control =====
    private final PIDController rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    // ===== Per-run State =====
    private double smoothedTx = 0.0;
    private boolean isFeeding = false;
    private double feedingElapsedSeconds = 0.0;

    /**
     * @param drivetrain           The swerve drivetrain
     * @param shooter              The shooter subsystem
     * @param vision               The vision subsystem
     * @param indexer              The indexer subsystem
     * @param shootDurationSeconds How long to feed once the shooter is ready (seconds)
     */
    public AutoAlignAndShootCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            IndexerSubsystem indexer,
            double shootDurationSeconds) {

        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.vision = vision;
        this.indexer = indexer;
        this.shootDurationSeconds = shootDurationSeconds;

        rotationPID = new PIDController(kP, kI, kD);
        rotationPID.setSetpoint(0.0);
        rotationPID.setTolerance(ALIGN_TOLERANCE_DEGREES);
        rotationPID.enableContinuousInput(-180.0, 180.0);

        driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, shooter, vision, indexer);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        isFeeding = false;
        feedingElapsedSeconds = 0.0;

        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            smoothedTx = vision.getHorizontalAngleDegrees();
            double distance = vision.getDistanceToTargetMeters();
            shooter.setTargetVelocity(interpolateRPM(distance));
            shooter.setTargetHoodPose(interpolateHoodAngle(distance));
        } else {
            smoothedTx = 0.0;
            shooter.setTargetVelocity(MIN_RPM);
            shooter.setTargetHoodPose(ShooterSubsystem.CLOSE_SHOT_HOOD);
        }

        shooter.prepareToShoot();
    }

    @Override
    public void execute() {
        // 1. Update smoothed tx
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double rawTx = vision.getHorizontalAngleDegrees();
            smoothedTx = (TX_FILTER_ALPHA * rawTx) + ((1.0 - TX_FILTER_ALPHA) * smoothedTx);
        }

        // 2. Update hood and RPM from live distance
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double distance = vision.getDistanceToTargetMeters();
            shooter.updateHoodForDistance(interpolateHoodAngle(distance));
            shooter.setTargetVelocity(interpolateRPM(distance));
        }

        // 3. Rotation PID — robot rotates to face hub, no driver translation
        double rotationOutput = 0.0;
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            if (Math.abs(smoothedTx) > ALIGN_TOLERANCE_DEGREES) {
                double pidOutput = rotationPID.calculate(smoothedTx);
                if (Math.abs(pidOutput) < MIN_ROTATION_OUTPUT) {
                    pidOutput = Math.copySign(MIN_ROTATION_OUTPUT, pidOutput);
                }
                rotationOutput = Math.max(-MAX_ROTATION_RATE,
                                 Math.min(MAX_ROTATION_RATE, pidOutput));
            }
        }

        // 4. Apply rotation only — robot stays stationary while aligning
        final double finalRotation = rotationOutput;
        drivetrain.applyRequest(() ->
            driveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(finalRotation)
        ).execute();

        // 5. Feed once shooter is ready, then count down shoot duration
        if (!isFeeding && shooter.isReady()) {
            isFeeding = true;
        }

        if (isFeeding) {
            feedingElapsedSeconds += 0.02; // 20ms robot loop period
            indexer.indexerForward();
            indexer.conveyorForward();
        } else {
            indexer.indexerStop();
            indexer.conveyorStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.indexerStop();
        indexer.conveyorStop();
        shooter.returnToIdle();
        rotationPID.reset();
    }

    @Override
    public boolean isFinished() {
        // Finish once we've been feeding for the configured duration
        return isFeeding && feedingElapsedSeconds >= shootDurationSeconds;
    }

    /**
     * Linearly interpolates flywheel RPM between MIN_RPM and MAX_RPM based on distance.
     * Clamped so distances outside the range pin to the nearest endpoint.
     */
    private double interpolateRPM(double distanceMeters) {
        double clamped = Math.max(CLOSE_DISTANCE_METERS,
                         Math.min(FAR_DISTANCE_METERS, distanceMeters));
        double t = (clamped - CLOSE_DISTANCE_METERS)
                 / (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);
        return MIN_RPM + t * (MAX_RPM - MIN_RPM);
    }

    /**
     * Linearly interpolates hood angle between CLOSE_SHOT_HOOD and FAR_SHOT_HOOD.
     * Clamped so distances outside the range pin to the nearest endpoint.
     */
    private double interpolateHoodAngle(double distanceMeters) {
        double clamped = Math.max(CLOSE_DISTANCE_METERS,
                         Math.min(FAR_DISTANCE_METERS, distanceMeters));
        double t = (clamped - CLOSE_DISTANCE_METERS)
                 / (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);
        return ShooterSubsystem.CLOSE_SHOT_HOOD
             + t * (ShooterSubsystem.FAR_SHOT_HOOD - ShooterSubsystem.CLOSE_SHOT_HOOD);
    }

    /**
     * Returns true if the tag ID is a valid hub target.
     */
    private boolean isHubTag(int tagId) {
        return HUB_TAG_IDS.contains(tagId);
    }
}