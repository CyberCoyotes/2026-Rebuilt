package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * AlignAndShootCommand — hub-aligned shooting with vision-primary / odometry-fallback rotation.
 *
 * Rotation source (re-evaluated every cycle):
 *   vision.hasFreshTarget() = true  → PID input is Limelight TX; drives TX → 0 so the
 *                                      camera (and shooter) centers directly on the tag.
 *   vision.hasFreshTarget() = false → PID input is odometry bearing to hub center; uses
 *                                      fused drivetrain pose, so MegaTag2 corrections still
 *                                      help even when the fallback path is active.
 *
 * Shot parameters (flywheel RPM and hood angle) are updated every cycle via
 * shooter.updateFromDistance(), which interpolates the preset lookup table using
 * the distance from the fused drivetrain pose to the hub.
 *
 * Feed gate: shooter.isReady() AND headingPID.atSetpoint().
 *
 * Use with whileTrue() on the shoot trigger.
 */
public class AlignAndShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final PIDController headingPID;
    private final SwerveRequest.FieldCentric alignRequest;

    // NT diagnostics — visible on Elastic under "AlignShoot/"
    private final DoublePublisher  ntTX;
    private final DoublePublisher  ntHeadingError;
    private final DoublePublisher  ntRotRate;
    private final DoublePublisher  ntDistance;
    private final BooleanPublisher ntAligned;
    private final BooleanPublisher ntShooterReady;
    private final StringPublisher  ntRotSource;

    public AlignAndShootCommand(
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        this.shooter    = shooter;
        this.indexer    = indexer;
        this.drivetrain = drivetrain;
        this.vision     = vision;
        this.xSupplier  = xSupplier;
        this.ySupplier  = ySupplier;

        headingPID = new PIDController(
                Constants.Vision.ROTATIONAL_KP,
                0.0,
                Constants.Vision.ROTATIONAL_KD);
        headingPID.enableContinuousInput(-180.0, 180.0);
        headingPID.setTolerance(Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES);

        alignRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        var nt     = NetworkTableInstance.getDefault().getTable("AlignShoot");
        ntTX           = nt.getDoubleTopic("tx_deg").publish();
        ntHeadingError = nt.getDoubleTopic("headingError_deg").publish();
        ntRotRate      = nt.getDoubleTopic("rotRate_radps").publish();
        ntDistance     = nt.getDoubleTopic("distance_m").publish();
        ntAligned      = nt.getBooleanTopic("aligned").publish();
        ntShooterReady = nt.getBooleanTopic("shooterReady").publish();
        ntRotSource    = nt.getStringTopic("rotSource").publish();

        addRequirements(shooter, indexer, drivetrain);
    }

    @Override
    public void initialize() {
        headingPID.reset();
        shooter.beginSpinUp();
    }

    @Override
    public void execute() {
        Pose2d pose        = drivetrain.getState().Pose;
        Translation2d hub  = getHubLocation();

        double dx       = hub.getX() - pose.getX();
        double dy       = hub.getY() - pose.getY();
        double distance = MathUtil.clamp(
                Math.hypot(dx, dy),
                Constants.Vision.MIN_DISTANCE_M,
                Constants.Vision.MAX_DISTANCE_M);

        // Live-update flywheel RPM and hood from interpolated distance table
        shooter.updateFromDistance(distance);
        if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
            shooter.beginSpinUp();
        }

        // ── Rotation target ─────────────────────────────────────────────────
        double currentHeadingDeg = pose.getRotation().getDegrees();
        double targetHeadingDeg;
        String rotSource;

        double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));
        if (vision.hasFreshTarget() && vision.getTagCount() == 1) {
            // Single hub tag: TX is a stable direct measurement — drive TX → 0.
            // Sign confirmed on robot: negative = correct for rear-mounted camera.
            targetHeadingDeg = MathUtil.inputModulus(
                    currentHeadingDeg - vision.getTX() + Constants.Vision.ALIGNMENT_OFFSET_DEGREES,
                    -180.0, 180.0);
            rotSource = "Vision/TX";
        } else {
            // Two or more hub tags visible: Limelight primary-tag selection can flip
            // between the pair, causing TX to jump 3–5°. MT2 pose with 2+ tags is
            // highly accurate, so odometry bearing is the more stable reference.
            // Also used as fallback when no hub tag is visible.
            targetHeadingDeg = MathUtil.inputModulus(
                    angleToHubDeg + Constants.Vision.ALIGNMENT_OFFSET_DEGREES,
                    -180.0, 180.0);
            rotSource = vision.hasFreshTarget() ? "Odometry/MultiTag" : "Odometry";
        }

        // ── PID → rotational rate ────────────────────────────────────────────
        headingPID.setSetpoint(targetHeadingDeg);
        double pidOutput      = headingPID.calculate(currentHeadingDeg);
        double headingErrorDeg = MathUtil.inputModulus(
                targetHeadingDeg - currentHeadingDeg, -180.0, 180.0);

        double rotRate;
        if (headingPID.atSetpoint()) {
            rotRate = 0.0;
        } else {
            // Enforce minimum to overcome static friction, then cap at maximum
            double sign      = Math.signum(pidOutput);
            double magnitude = Math.max(
                    Math.abs(pidOutput),
                    Constants.Vision.MIN_ALIGNMENT_ROTATION_RAD_PER_SEC);
            rotRate = MathUtil.clamp(
                    sign * magnitude,
                    -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                    Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);
        }

        drivetrain.setControl(
                alignRequest
                        .withVelocityX(-xSupplier.getAsDouble() * Constants.Vision.ALIGNMENT_DRIVETRAIN_CLAMP)
                        .withVelocityY(-ySupplier.getAsDouble() * Constants.Vision.ALIGNMENT_DRIVETRAIN_CLAMP)
                        .withRotationalRate(rotRate));

        // ── Feed gate ────────────────────────────────────────────────────────
        // ALIGNMENT_TOLERANCE_DEGREES controls the rotation deadband (PID stops turning).
        // FEED_TOLERANCE_DEGREES is checked independently and is intentionally looser —
        // the shot fires when we're "close enough," not only when perfectly settled.
        // This prevents the robot from never feeding because it can't hold sub-1° alignment.
        boolean aligned  = headingPID.atSetpoint(); // used for dashboard only
        boolean readyToFeed = Math.abs(headingErrorDeg) < Constants.Vision.FEED_TOLERANCE_DEGREES;
        if (shooter.isReady() && readyToFeed) {
            indexer.conveyorForward();
            indexer.kickerForward();
        } else {
            indexer.indexerStop();
            indexer.conveyorStop();
        }

        // ── Diagnostics ──────────────────────────────────────────────────────
        ntTX.set(vision.getTX());
        ntHeadingError.set(headingErrorDeg);
        ntRotRate.set(rotRate);
        ntDistance.set(distance);
        ntAligned.set(aligned);
        ntShooterReady.set(shooter.isReady());
        ntRotSource.set(rotSource);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until trigger is released (whileTrue)
    }

    @Override
    public void end(boolean interrupted) {
        indexer.indexerStop();
        indexer.conveyorStop();
        shooter.setPostShotState();
    }

    private static Translation2d getHubLocation() {
        return DriverStation.getAlliance()
                .filter(a -> a == DriverStation.Alliance.Red)
                .map(a -> Constants.Vision.RED_HUB_LOCATION)
                .orElse(Constants.Vision.BLUE_HUB_LOCATION);
    }
}
