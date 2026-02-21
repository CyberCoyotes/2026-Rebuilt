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
 * FarShotCommand - Dynamic far shot with auto-rotate and distance-based hood adjustment.
 *
 * BEHAVIOR (while RT is held):
 * - Auto-rotates to face the nearest hub AprilTag (tags 18-27) using PID on tx
 * - Continuously updates hood angle based on live distance from Limelight
 * - RPM fixed at FAR_SHOT_RPM + 350 for extra margin
 * - Hood interpolates between CLOSE_SHOT_HOOD (at 0.5m) and FAR_SHOT_HOOD (at 4.0m)
 * - Driver retains full translational control via left stick throughout
 * - If tag is lost mid-hold, freezes last known distance/angle from THIS hold only
 *   (does not use stale data from a previous press)
 * - On release: stops indexer, returns shooter to SPINUP at IDLE_RPM
 *
 * TUNING:
 * - kP: Rotation PID gain — increase if slow to snap, decrease if oscillating
 * - CLOSE_DISTANCE_METERS / FAR_DISTANCE_METERS: Distance endpoints for interpolation
 * - FAR_SHOT_RPM_BOOST: Extra RPM on top of FAR_SHOT_RPM for safety margin
 *
 * USAGE:
 *   driver.rightTrigger(0.5).whileTrue(new FarShotCommand(drivetrain, shooter, vision, indexer,
 *       () -> -driver.getLeftY() * MaxSpeed,
 *       () -> -driver.getLeftX() * MaxSpeed));
 *
 * @author @Isaak3
 */
public class FarShotCommand extends Command {

    // ===== Hub Tag Filter =====
    private static final int MIN_HUB_TAG_ID = 18;
    private static final int MAX_HUB_TAG_ID = 27;

    // ===== Rotation PID =====
    private static final double kP = 0.05;   // TODO: Tune — same as AlignToHubCommand
    private static final double kI = 0.0;
    private static final double kD = 0.001;
    private static final double ALIGN_TOLERANCE_DEGREES = 2.0;
    private static final double MAX_ROTATION_RATE = 4.0;

    // ===== Distance Interpolation =====
    /** Distance (meters) that maps to CLOSE_SHOT_HOOD (minimum hood angle) */
    private static final double CLOSE_DISTANCE_METERS = 0.5; // TODO: Tune

    /** Distance (meters) that maps to FAR_SHOT_HOOD (maximum hood angle) */
    private static final double FAR_DISTANCE_METERS = 4.0; // TODO: Tune

    /** Extra RPM added on top of FAR_SHOT_RPM for safety margin */
    private static final double FAR_SHOT_RPM_BOOST = 350.0;

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
    // These reset on each new press — no stale data from previous holds
    private double lastKnownDistance = 0.0;
    private boolean hasSeenTagThisHold = false;
    private boolean isFeeding = false;

    /**
     * Creates a new FarShotCommand.
     *
     * @param drivetrain   The swerve drivetrain subsystem
     * @param shooter      The shooter subsystem
     * @param vision       The vision subsystem
     * @param indexer      The indexer subsystem
     * @param translationX Supplier for forward/back velocity (left stick Y, negated and scaled)
     * @param translationY Supplier for strafe velocity (left stick X, negated and scaled)
     */
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
        // Reset all per-hold state — clean slate on each new press
        rotationPID.reset();
        lastKnownDistance = 0.0;
        hasSeenTagThisHold = false;
        isFeeding = false;

        // Set flywheel to far shot RPM + boost immediately
        shooter.setTargetVelocity(ShooterSubsystem.FAR_SHOT_RPM + FAR_SHOT_RPM_BOOST);
        shooter.prepareToShoot();
    }

    @Override
    public void execute() {
        // =====================================================================
        // 1. Update distance and hood from vision
        // =====================================================================
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double distance = vision.getDistanceToTargetMeters();
            lastKnownDistance = distance;
            hasSeenTagThisHold = true;

            // Interpolate hood angle from distance and command hood to move immediately
            double hoodAngle = interpolateHoodAngle(distance);
            shooter.updateHoodForDistance(hoodAngle);
        }
        // If tag lost, use lastKnownDistance from this hold — no updates, no resets

        // =====================================================================
        // 2. Auto-rotate toward hub tag
        // =====================================================================
        double rotationOutput = 0.0;

        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double txDegrees = vision.getHorizontalAngleDegrees();
            if (Math.abs(txDegrees) > ALIGN_TOLERANCE_DEGREES) {
                rotationOutput = rotationPID.calculate(txDegrees);
                rotationOutput = Math.max(-MAX_ROTATION_RATE, Math.min(MAX_ROTATION_RATE, rotationOutput));
            }
        }

        // =====================================================================
        // 3. Drive — pass through driver translation, override rotation with PID
        // =====================================================================
        final double finalRotation = rotationOutput;
        drivetrain.applyRequest(() ->
            driveRequest
                .withVelocityX(translationX.getAsDouble())
                .withVelocityY(translationY.getAsDouble())
                .withRotationalRate(finalRotation)
        ).execute();

        // =====================================================================
        // 4. Feed once shooter is ready
        // =====================================================================
        if (shooter.isReady()) {
            isFeeding = true;
            indexer.indexerForward();
            indexer.conveyorForward();
        } else if (!isFeeding) {
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
        return false; // Run until button released
    }

    /**
     * Linearly interpolates hood angle between CLOSE_SHOT_HOOD and FAR_SHOT_HOOD
     * based on distance. Clamped to the valid range.
     *
     * At CLOSE_DISTANCE_METERS → CLOSE_SHOT_HOOD
     * At FAR_DISTANCE_METERS  → FAR_SHOT_HOOD
     *
     * @param distanceMeters Distance to hub in meters
     * @return Interpolated hood angle in rotations
     */
    private double interpolateHoodAngle(double distanceMeters) {
        // Clamp distance to our known range
        double clampedDistance = Math.max(CLOSE_DISTANCE_METERS,
                                 Math.min(FAR_DISTANCE_METERS, distanceMeters));

        // Normalize to 0.0 - 1.0
        double t = (clampedDistance - CLOSE_DISTANCE_METERS)
                 / (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);

        // Interpolate between close and far hood positions
        return ShooterSubsystem.CLOSE_SHOT_HOOD
             + t * (ShooterSubsystem.FAR_SHOT_HOOD - ShooterSubsystem.CLOSE_SHOT_HOOD);
    }

    /**
     * Returns true if the given tag ID belongs to the blue hub (tags 18-27).
     */
    private boolean isHubTag(int tagId) {
        return tagId >= MIN_HUB_TAG_ID && tagId <= MAX_HUB_TAG_ID;
    }
}