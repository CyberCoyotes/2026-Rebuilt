package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

/**
 * FarShotCommand - Auto-aligns to hub, adjusts hood and RPM based on distance.
 *
 * BEHAVIOR (while RT is held):
 * - Auto-rotates to face the nearest hub AprilTag (tags 18-27, excluding 17, 22, 23, 28)
 * - tx is smoothed with an EMA filter to reject tag-switching noise and vibration
 * - Continuously interpolates both hood angle AND flywheel RPM from live distance:
 *     At CLOSE_DISTANCE_METERS (0.85m): CLOSE_SHOT_HOOD + MIN_RPM (2250)
 *     At FAR_DISTANCE_METERS  (3.5m):  FAR_SHOT_HOOD  + MAX_RPM (3250)
 * - Driver retains full translational control throughout
 * - If tag is lost mid-hold, hood/RPM/rotation freeze at last known values for THIS hold only
 * - Feeding and alignment run simultaneously — feeds as soon as shooter is ready
 * - On release: stops indexer, returns shooter to SPINUP at IDLE_RPM
 *
 * TUNING:
 * - TX_FILTER_ALPHA: EMA smoothing (0.0 = frozen, 1.0 = raw). Decrease to reduce jitter.
 * - kP / MIN_ROTATION_OUTPUT / MAX_ROTATION_RATE: Rotation PID tuning
 * - CLOSE_DISTANCE_METERS / FAR_DISTANCE_METERS: Distance endpoints for interpolation
 * - MIN_RPM / MAX_RPM: RPM endpoints — MIN at close range, MAX at far range
 *
 * @author @Isaak3
 */
public class FarShotCommand extends Command {

    // ===== Hub Tag Filter =====
    private static final int MIN_HUB_TAG_ID = 18;
    private static final int MAX_HUB_TAG_ID = 27;

    // ===== Distance Interpolation Range =====
    /** Closest distance (meters) — maps to CLOSE_SHOT_HOOD and MIN_RPM */
    private static final double CLOSE_DISTANCE_METERS = 0.85;

    /** Farthest distance (meters) — maps to FAR_SHOT_HOOD and MAX_RPM */
    private static final double FAR_DISTANCE_METERS = 3.5;

    // ===== RPM Interpolation =====
    /** Flywheel RPM at closest distance */
    private static final double MIN_RPM = 2875; // TODO: Tune

    /** Flywheel RPM at farthest distance */
    private static final double MAX_RPM = 3875; // TODO: Tune

    // ===== tx EMA Filter =====
    /**
     * Exponential moving average smoothing factor for tx.
     * Filters noise from tag switching and mechanical vibration during shooting.
     * Range: 0.0 (fully frozen) to 1.0 (no filtering, raw tx).
     */
    private static final double TX_FILTER_ALPHA = 0.2; // TODO: Tune

    // ===== Rotation PID =====
    private static final double kP = 0.05; // TODO: Tune
    private static final double kI = 0.0;
    private static final double kD = 0.001;

    /**
     * Minimum rotation output (rad/s) applied whenever error exceeds the deadband.
     * Ensures robot always commits to rotating even when PID output is weak at large errors.
     */
    private static final double MIN_ROTATION_OUTPUT = 1.25; // TODO: Tune

    /** Maximum rotation rate (rad/s) */
    private static final double MAX_ROTATION_RATE = 4.0; // TODO: Tune

    /** Deadband — within this many degrees of smoothed tx, no correction applied */
    private static final double ALIGN_TOLERANCE_DEGREES = 1.5;

    // ===== Hardware =====
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;

    // ===== Control =====
    private final PIDController rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    // ===== Per-hold State =====
    private boolean isFeeding = false;
    private double smoothedTx = 0.0;

    public FarShotCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            IndexerSubsystem indexer,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.vision = vision;
        this.indexer = indexer;
        this.translationX = translationX;
        this.translationY = translationY;

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

        // Seed smoothedTx from raw tx so the filter starts from a meaningful value
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

        // Enter READY state — flywheel ramps up, hood moves to target
        shooter.prepareToShoot();
    }

    @Override
    public void execute() {
        // =====================================================================
        // 1. Update smoothed tx via EMA filter
        // =====================================================================
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double rawTx = vision.getHorizontalAngleDegrees();
            smoothedTx = (TX_FILTER_ALPHA * rawTx) + ((1.0 - TX_FILTER_ALPHA) * smoothedTx);
        }

        // =====================================================================
        // 2. Update hood and RPM continuously from live distance
        // =====================================================================
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double distance = vision.getDistanceToTargetMeters();
            shooter.updateHoodForDistance(interpolateHoodAngle(distance));
            shooter.setTargetVelocity(interpolateRPM(distance));
        }
        // If no valid tag — hood and RPM hold their last commanded values

        // =====================================================================
        // 3. Rotation PID on smoothed tx with minimum output floor
        // =====================================================================
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

        // =====================================================================
        // 4. Drive — driver translation + PID rotation
        // =====================================================================
        final double finalRotation = rotationOutput;
        drivetrain.applyRequest(() ->
            driveRequest
                .withVelocityX(translationX.getAsDouble())
                .withVelocityY(translationY.getAsDouble())
                .withRotationalRate(finalRotation)
        ).execute();

        // =====================================================================
        // 5. Feed once shooter is ready
        // =====================================================================
        if (shooter.isReady()) {
            isFeeding = true;
            // indexer.indexerForward();
            // indexer.conveyorForward();
        } else if (!isFeeding) {
            // indexer.indexerStop();
            // indexer.conveyorStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // indexer.indexerStop();
        // indexer.conveyorStop();
        shooter.returnToIdle();
        rotationPID.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Linearly interpolates flywheel RPM between MIN_RPM and MAX_RPM based on distance.
     * Clamped so distances outside the range pin to the nearest endpoint.
     *
     * At CLOSE_DISTANCE_METERS → MIN_RPM (2250)
     * At FAR_DISTANCE_METERS  → MAX_RPM (3250)
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
     *
     * At CLOSE_DISTANCE_METERS → CLOSE_SHOT_HOOD
     * At FAR_DISTANCE_METERS  → FAR_SHOT_HOOD
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
     * Must be within the hub range (18-27) and not in the exclusion list.
     *
     * Excluded tags: 17, 22, 23, 28
     */
    private boolean isHubTag(int tagId) {
        if (tagId == 17 || tagId == 22 || tagId == 23 || tagId == 28) {
            return false;
        }
        return tagId >= MIN_HUB_TAG_ID && tagId <= MAX_HUB_TAG_ID;
    }
}