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
 * FarShotCommand - Auto-aligns to hub and adjusts hood angle based on distance.
 *
 * BEHAVIOR (while RT is held):
 * - Flywheel spins at a fixed 3000 RPM
 * - Auto-rotates to face the nearest hub AprilTag (tags 18-27) using PID on tx
 * - Continuously updates hood angle based on live distance, interpolating between
 *   CLOSE_SHOT_HOOD at CLOSE_DISTANCE_METERS and FAR_SHOT_HOOD at FAR_DISTANCE_METERS
 * - Driver retains full translational control throughout
 * - If tag is lost mid-hold, hood and rotation freeze at last known values for THIS hold only
 * - Once flywheel and hood are ready, feeds the game piece automatically
 * - On release: stops indexer, returns shooter to SPINUP at IDLE_RPM
 *
 * TUNING:
 * - kP / kD: Rotation PID — increase kP if slow to snap, add kD to dampen oscillation
 * - CLOSE_DISTANCE_METERS / FAR_DISTANCE_METERS: Distance range for hood interpolation
 * - FIXED_RPM: Flywheel target during this command
 *
 * @author @Isaak3
 */
public class FarShotCommand extends Command {

    // ===== Hub Tag Filter =====
    private static final int MIN_HUB_TAG_ID = 18;
    private static final int MAX_HUB_TAG_ID = 27;

    // ===== Flywheel =====
    /** Fixed RPM for this command */
    private static final double FIXED_RPM = 3000.0; // TODO: Tune

    // ===== Rotation PID =====
    private static final double kP = 0.05;  // TODO: Tune
    private static final double kI = 0.0;
    private static final double kD = 0.001; // TODO: Tune
    private static final double ALIGN_TOLERANCE_DEGREES = 3.0;
    private static final double MAX_ROTATION_RATE = 4.0; // rad/s

    // ===== Hood Distance Interpolation =====
    /** Distance in meters that maps to CLOSE_SHOT_HOOD (minimum angle) */
    private static final double CLOSE_DISTANCE_METERS = 1.0;

    /** Distance in meters that maps to FAR_SHOT_HOOD (maximum angle) */
    private static final double FAR_DISTANCE_METERS = 2.6;

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

        // Fixed RPM — never changes during this command
        shooter.setTargetVelocity(FIXED_RPM);

        // Set initial hood angle from current distance if a tag is already visible,
        // otherwise fall back to CLOSE_SHOT_HOOD — execute() will update it immediately
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            shooter.setTargetHoodPose(interpolateHoodAngle(vision.getDistanceToTargetMeters()));
        } else {
            shooter.setTargetHoodPose(ShooterSubsystem.CLOSE_SHOT_HOOD);
        }

        // Enter READY state — flywheel ramps up, hood moves to the target set above
        shooter.prepareToShoot();
    }

    @Override
    public void execute() {
        // =====================================================================
        // 1. Update hood angle continuously from live distance
        // =====================================================================
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double hoodAngle = interpolateHoodAngle(vision.getDistanceToTargetMeters());
            // Pushes new target to shooter and immediately commands hood motor
            shooter.updateHoodForDistance(hoodAngle);
        }
        // If no tag visible — hood stays at last commanded position

        // =====================================================================
        // 2. Rotation PID toward tx = 0
        // =====================================================================
        double rotationOutput = 0.0;
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double txDegrees = vision.getHorizontalAngleDegrees();
            if (Math.abs(txDegrees) > ALIGN_TOLERANCE_DEGREES) {
                rotationOutput = rotationPID.calculate(txDegrees);
                rotationOutput = Math.max(-MAX_ROTATION_RATE,
                                 Math.min(MAX_ROTATION_RATE, rotationOutput));
            }
        }

        // =====================================================================
        // 3. Drive — driver translation + PID rotation
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
        return false;
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

    private boolean isHubTag(int tagId) {
        return tagId >= MIN_HUB_TAG_ID && tagId <= MAX_HUB_TAG_ID;
    }
}