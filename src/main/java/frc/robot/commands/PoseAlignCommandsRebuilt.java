package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Standalone pose-alignment command based on today's poseAlignAndShoot behavior.
 *
 * <p>This keeps the current odometry-based angle math, lead compensation, rotation
 * control, and reduced driver translation scaling, but does not touch shooter or
 * indexer state.
 */
public final class PoseAlignCommandsRebuilt {
    private PoseAlignCommandsRebuilt() {}

    /** Returns the hub center for the current alliance (defaults to blue if FMS not connected). */
    private static Translation2d getHubLocation() {
        return DriverStation.getAlliance()
                .filter(a -> a == DriverStation.Alliance.Red)
                .map(a -> Constants.Vision.RED_HUB_LOCATION)
                .orElse(Constants.Vision.BLUE_HUB_LOCATION);
    }

    /**
     * Today's standalone pose alignment extracted from the live poseAlignAndShoot command.
     *
     * <p>Driver keeps translation control while the command owns rotational aiming.
     */
    public static Command poseAlignRebuilt(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("VisionAlignRebuilt");
        final DoublePublisher ntAngleToHub = visionTable.getDoubleTopic("angleToHub_deg").publish();
        final DoublePublisher ntCurrentHeading = visionTable.getDoubleTopic("currentHeading_deg").publish();
        final DoublePublisher ntHeadingError = visionTable.getDoubleTopic("headingError_deg").publish();
        final DoublePublisher ntRotRate = visionTable.getDoubleTopic("rotRate_radps").publish();
        final DoublePublisher ntDistance = visionTable.getDoubleTopic("distanceToHub_m").publish();
        final DoublePublisher ntLeadOffset = visionTable.getDoubleTopic("leadOffset_deg").publish();

        return Commands.run(() -> {
            Translation2d hub = getHubLocation();
            Pose2d pose = drivetrain.getState().Pose;

            double dx = hub.getX() - pose.getX();
            double dy = hub.getY() - pose.getY();
            double distance = MathUtil.clamp(
                    Math.hypot(dx, dy),
                    Constants.Vision.MIN_DISTANCE_M,
                    Constants.Vision.MAX_DISTANCE_M);

            double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));

            var speeds = drivetrain.getState().Speeds;
            double vx = speeds.vxMetersPerSecond;
            double vy = speeds.vyMetersPerSecond;
            double hubAngleRad = Math.atan2(dy, dx);
            double lateralVelocity = -vx * Math.sin(hubAngleRad) + vy * Math.cos(hubAngleRad);
            double leadOffsetDeg = -lateralVelocity * Constants.Vision.LEAD_COMPENSATION_DEG_PER_MPS;

            double currentHeadingDeg = pose.getRotation().getDegrees();
            double headingErrorDeg = angleToHubDeg + leadOffsetDeg - currentHeadingDeg;
            while (headingErrorDeg > 180) headingErrorDeg -= 360;
            while (headingErrorDeg < -180) headingErrorDeg += 360;

            double rotRate = MathUtil.clamp(
                    headingErrorDeg * Constants.Vision.ROTATIONAL_KP,
                    -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                    Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);

            ntAngleToHub.set(angleToHubDeg);
            ntCurrentHeading.set(currentHeadingDeg);
            ntHeadingError.set(headingErrorDeg);
            ntRotRate.set(rotRate);
            ntDistance.set(distance);
            ntLeadOffset.set(leadOffsetDeg);

            drivetrain.setControl(
                    alignRequest
                            .withVelocityX(xSupplier.getAsDouble() * .40)
                            .withVelocityY(ySupplier.getAsDouble() * .40)
                            .withRotationalRate(rotRate));
        }, drivetrain).withName("PoseAlignRebuilt");
    }
}
