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
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * AlignOnlyCommand — rotation-only version of AlignAndShootCommand for tuning PID gains
 * without spinning up the flywheel or moving the hood.
 *
 * Identical rotation logic and NT diagnostics (AlignShoot/ table) — swap in place of
 * AlignAndShootCommand on a test button, compare Elastic side-by-side.
 *
 * Does NOT require shooter or indexer so those subsystems stay cool.
 */
public class AlignOnlyCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final PIDController headingPID;
    private final SwerveRequest.FieldCentric alignRequest;

    // Same NT keys as AlignAndShootCommand for direct Elastic comparison
    private final DoublePublisher  ntTX;
    private final DoublePublisher  ntHeadingError;
    private final DoublePublisher  ntRotRate;
    private final DoublePublisher  ntDistance;
    private final BooleanPublisher ntAligned;
    private final StringPublisher  ntRotSource;

    public AlignOnlyCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

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

        var nt = NetworkTableInstance.getDefault().getTable("AlignShoot");
        ntTX           = nt.getDoubleTopic("tx_deg").publish();
        ntHeadingError = nt.getDoubleTopic("headingError_deg").publish();
        ntRotRate      = nt.getDoubleTopic("rotRate_radps").publish();
        ntDistance     = nt.getDoubleTopic("distance_m").publish();
        ntAligned      = nt.getBooleanTopic("aligned").publish();
        ntRotSource    = nt.getStringTopic("rotSource").publish();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        headingPID.reset();
    }

    @Override
    public void execute() {
        Pose2d pose       = drivetrain.getState().Pose;
        Translation2d hub = getHubLocation();

        double dx       = hub.getX() - pose.getX();
        double dy       = hub.getY() - pose.getY();
        double distance = MathUtil.clamp(
                Math.hypot(dx, dy),
                Constants.Vision.MIN_DISTANCE_M,
                Constants.Vision.MAX_DISTANCE_M);

        double currentHeadingDeg = pose.getRotation().getDegrees();
        double targetHeadingDeg;
        String rotSource;

        double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));
        if (vision.hasFreshTarget() && vision.getTagCount() == 1) {
            targetHeadingDeg = MathUtil.inputModulus(
                    currentHeadingDeg - vision.getTX() + Constants.Vision.ALIGNMENT_OFFSET_DEGREES,
                    -180.0, 180.0);
            rotSource = "Vision/TX";
        } else {
            targetHeadingDeg = MathUtil.inputModulus(
                    angleToHubDeg + Constants.Vision.ALIGNMENT_OFFSET_DEGREES,
                    -180.0, 180.0);
            rotSource = vision.hasFreshTarget() ? "Odometry/MultiTag" : "Odometry";
        }

        headingPID.setSetpoint(targetHeadingDeg);
        double pidOutput       = headingPID.calculate(currentHeadingDeg);
        double headingErrorDeg = MathUtil.inputModulus(
                targetHeadingDeg - currentHeadingDeg, -180.0, 180.0);

        double rotRate;
        if (headingPID.atSetpoint()) {
            rotRate = 0.0;
        } else {
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

        ntTX.set(vision.getTX());
        ntHeadingError.set(headingErrorDeg);
        ntRotRate.set(rotRate);
        ntDistance.set(distance);
        ntAligned.set(headingPID.atSetpoint());
        ntRotSource.set(rotSource);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // No shooter or indexer to stop — drivetrain returns to default command automatically
    }

    private static Translation2d getHubLocation() {
        return DriverStation.getAlliance()
                .filter(a -> a == DriverStation.Alliance.Red)
                .map(a -> Constants.Vision.RED_HUB_LOCATION)
                .orElse(Constants.Vision.BLUE_HUB_LOCATION);
    }
}
