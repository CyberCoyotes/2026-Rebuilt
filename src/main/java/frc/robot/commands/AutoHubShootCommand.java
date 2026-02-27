package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * AutoHubShootCommand - Fully automatic hub-targeting and shooting command.
 *
 * BEHAVIOR (while held):
 *   1. Calculates distance from robot's MegaTag2 pose to the hub center
 *   2. Interpolates flywheel RPM and hood angle from that distance
 *   3. Snaps robot rotation to face the hub using a ProfiledPIDController
 *   4. Spins flywheel and moves hood to interpolated targets
 *   5. Feeds indexer/conveyor ONLY when:
 *        - Shooter is at target RPM and hood angle (isReady())
 *        - Robot is rotationally aligned to hub within 1.5 degrees
 *        - MegaTag2 pose estimate is valid
 *   6. If MegaTag2 is lost mid-shot:
 *        - Stops feeding immediately
 *        - Keeps spinning at last known targets
 *        - Resumes feeding automatically when pose returns
 *   7. On button release: stops indexer/conveyor, stops shooter, ends command
 *
 * INTERPOLATION:
 *   - Distance is clamped to [MIN_DISTANCE_M, MAX_DISTANCE_M]
 *   - RPM and hood angle scale linearly across that range
 *   - At 0.5m → 2875 RPM, hood 0
 *   - At 4.0m → 4000 RPM, hood 9
 *
 * TUNING:
 *   - kP, kD:              Rotation PID gains
 *   - MAX_VELOCITY:        Max rotation speed during snap (rad/s)
 *   - MAX_ACCEL:           Max rotation acceleration (rad/s²)
 *   - ALIGN_TOLERANCE_DEG: Deadband for "aligned" check (degrees)
 */
public class AutoHubShootCommand extends Command {

    // ===== Hub Position (WPILib Blue field coordinates, meters) =====
    private static final Translation2d HUB_CENTER = new Translation2d(4.625, 4.05);

    // ===== Interpolation Range =====
    private static final double MIN_DISTANCE_M      = 0.5;
    private static final double MAX_DISTANCE_M      = 4.0;

    private static final double MIN_RPM             = 2875.0;
    private static final double MAX_RPM             = 4000.0;

    private static final double MIN_HOOD            = 0.0;
    private static final double MAX_HOOD            = 9.0;

    // ===== Rotation Alignment =====
    private static final double ALIGN_TOLERANCE_DEG = 1.5;

    private static final double kP                  = 5.0;   // TODO: Tune
    private static final double kD                  = 0.1;   // TODO: Tune
    private static final double MAX_VELOCITY        = 4.0;   // rad/s, TODO: Tune
    private static final double MAX_ACCEL           = 6.0;   // rad/s², TODO: Tune

    // ===== Hardware =====
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;

    // ===== Control =====
    private final ProfiledPIDController rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    /** Last valid goal angle — held if MegaTag2 briefly drops out */
    private double lastGoalAngleRadians = 0.0;
    private boolean hasHadFirstEstimate = false;

    /** Last valid interpolated targets — held while spinning if pose is lost */
    private double lastTargetRPM  = MIN_RPM;
    private double lastTargetHood = MIN_HOOD;

    public AutoHubShootCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.drivetrain   = drivetrain;
        this.vision       = vision;
        this.shooter      = shooter;
        this.indexer      = indexer;
        this.translationX = translationX;
        this.translationY = translationY;

        rotationPID = new ProfiledPIDController(
            kP, 0.0, kD,
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL)
        );
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Math.toRadians(ALIGN_TOLERANCE_DEG));

        driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Vision is a sensor — never add it to requirements
        addRequirements(drivetrain, shooter, indexer);
    }

    @Override
    public void initialize() {
        rotationPID.reset(drivetrain.getState().Pose.getRotation().getRadians());
        hasHadFirstEstimate = false;
        lastTargetRPM  = MIN_RPM;
        lastTargetHood = MIN_HOOD;

        // Start spinning immediately at minimum so flywheel isn't cold on first shot
        shooter.setTargetVelocity(MIN_RPM);
        shooter.setTargetHoodPose(MIN_HOOD);
        shooter.prepareToShoot();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        boolean hasPose  = vision.hasMegaTag2Estimate();

        // ── 1. Distance → Interpolated Targets ───────────────────────────────
        if (hasPose) {
            // Calculate straight-line distance from robot to hub center
            double distance = robotPose.getTranslation().getDistance(HUB_CENTER);

            // Clamp distance to our known interpolation range
            double clampedDist = MathUtil.clamp(distance, MIN_DISTANCE_M, MAX_DISTANCE_M);

            // Linear interpolation: t=0 at min distance, t=1 at max distance
            double t = (clampedDist - MIN_DISTANCE_M) / (MAX_DISTANCE_M - MIN_DISTANCE_M);

            lastTargetRPM  = MIN_RPM  + t * (MAX_RPM  - MIN_RPM);
            lastTargetHood = MIN_HOOD + t * (MAX_HOOD - MIN_HOOD);

            // Calculate angle to hub for rotation snap
            Translation2d toHub = HUB_CENTER.minus(robotPose.getTranslation());
            lastGoalAngleRadians = Math.atan2(toHub.getY(), toHub.getX());
            hasHadFirstEstimate  = true;
        }
        // If pose is lost, lastTargetRPM/Hood and lastGoalAngleRadians hold their
        // last valid values — shooter keeps spinning, rotation holds last angle

        // ── 2. Push Targets to Shooter ────────────────────────────────────────
        shooter.setTargetVelocity(lastTargetRPM);
        shooter.setTargetHoodPose(lastTargetHood);
        if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
            shooter.prepareToShoot();
        }

        // ── 3. Rotation Snap to Hub ───────────────────────────────────────────
        double rotationOutput = 0.0;
        if (hasHadFirstEstimate) {
            rotationPID.setGoal(lastGoalAngleRadians);
            rotationOutput = rotationPID.calculate(robotPose.getRotation().getRadians());
        }

        drivetrain.setControl(
            driveRequest
                .withVelocityX(translationX.getAsDouble())
                .withVelocityY(translationY.getAsDouble())
                .withRotationalRate(rotationOutput)
        );

        // ── 4. Feed Gate ──────────────────────────────────────────────────────
        // All three conditions must be true simultaneously:
        //   - Valid pose (we know where we are)
        //   - Rotationally aligned to hub within 1.5 degrees
        //   - Shooter at target RPM and hood angle
        boolean aligned = hasHadFirstEstimate && rotationPID.atGoal();
        boolean ready   = shooter.isReady();

        if (hasPose && aligned && ready) {
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
        shooter.setIdle();
        rotationPID.reset(drivetrain.getState().Pose.getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button is released
    }

    /**
     * Returns true when all conditions for firing are met.
     * Useful for LED feedback or dashboard indicators.
     */
    public boolean isReadyToFire() {
        return vision.hasMegaTag2Estimate()
            && hasHadFirstEstimate
            && rotationPID.atGoal()
            && shooter.isReady();
    }
}