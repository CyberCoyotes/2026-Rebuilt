package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

/**
 * SnapToHubCommand - Continuously rotates the robot to face the hub center
 * using field-relative pose from MegaTag2.
 *
 * APPROACH:
 * - Uses the robot's known field position (from MegaTag2 odometry) to
 *   mathematically calculate the angle to the hub center
 * - Drives a ProfiledPIDController toward that angle every loop
 * - Driver retains full translational control throughout
 * - Falls back to holding last known angle if MegaTag2 estimate is lost
 *
 * TUNING:
 * - kP: Increase if slow to snap, decrease if oscillating
 * - kD: Increase slightly if overshooting
 * - MAX_VELOCITY / MAX_ACCEL: Controls how aggressively the robot rotates.
 *   Lower these if the snap feels violent.
 * - ALIGN_TOLERANCE_DEGREES: How close is "close enough" for isAligned()
 */
public class SnapToHubCommand extends Command {

    // ===== Hub Position (WPILib Blue field coordinates) =====
    private static final Translation2d HUB_CENTER = new Translation2d(4.625, 4.05);

    // ===== PID / Motion Profile =====
    private static final double kP = 5.0;   // TODO: Tune on robot
    private static final double kI = 0.0;
    private static final double kD = 0.1;   // TODO: Tune on robot

    private static final double MAX_VELOCITY_RAD_PER_SEC = 4.0;  // TODO: Tune
    private static final double MAX_ACCEL_RAD_PER_SEC_SQ = 6.0;  // TODO: Tune

    /** Within this many degrees, the robot is considered aligned */
    private static final double ALIGN_TOLERANCE_DEGREES = 2.0;

    // ===== Hardware =====
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;

    // ===== Control =====
    private final ProfiledPIDController rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    /** Last valid goal angle — held if MegaTag2 estimate is temporarily lost */
    private double lastGoalAngleRadians = 0.0;
    private boolean hasHadFirstEstimate = false;

    public SnapToHubCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.drivetrain = drivetrain;
        this.vision = vision;
        this.translationX = translationX;
        this.translationY = translationY;

        rotationPID = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(MAX_VELOCITY_RAD_PER_SEC, MAX_ACCEL_RAD_PER_SEC_SQ)
        );
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Math.toRadians(ALIGN_TOLERANCE_DEGREES));

        driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Only require drivetrain — vision is a sensor, not an actuator
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationPID.reset(drivetrain.getState().Pose.getRotation().getRadians());
        hasHadFirstEstimate = false;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;

        if (vision.hasMegaTag2Estimate()) {
            // Calculate the angle from the robot's current position to the hub center
            Translation2d toHub = HUB_CENTER.minus(robotPose.getTranslation());
            double angleToHub = Math.atan2(toHub.getY(), toHub.getX());

            lastGoalAngleRadians = angleToHub;
            hasHadFirstEstimate = true;
        }
        // If no estimate yet, hasHadFirstEstimate is false and we skip rotation correction

        double rotationOutput = 0.0;
        if (hasHadFirstEstimate) {
            rotationPID.setGoal(lastGoalAngleRadians);
            rotationOutput = rotationPID.calculate(robotPose.getRotation().getRadians());
        }

        final double finalRotation = rotationOutput;
        drivetrain.setControl(
            driveRequest
                .withVelocityX(translationX.getAsDouble())
                .withVelocityY(translationY.getAsDouble())
                .withRotationalRate(finalRotation)
        );
    }

    @Override
    public void end(boolean interrupted) {
        rotationPID.reset(drivetrain.getState().Pose.getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button is released
    }

    /**
     * Returns true when the robot is pointed at the hub within tolerance.
     * Useful for gating indexer feed in a shoot command.
     */
    public boolean isAligned() {
        return hasHadFirstEstimate && rotationPID.atGoal();
    }
}