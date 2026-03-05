package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.DoubleSupplier;

/**
 * FarShotCommand - Pose-based far shot with auto-rotation toward hub.
 *
 * Behavior (while held):
 * 1. Gets the robot's current field pose from drivetrain odometry.
 * 2. Calculates distance and angle to the hub using field coordinates.
 * 3. Calls updateFromDistance() every loop — pushes RPM and hood to hardware continuously.
 * 4. Rotates the drivetrain to face the hub (driver can still translate freely).
 * 5. Once aligned within 0.5 degrees, feeds the indexer.
 * 6. On release: stops indexer, conveyor, and shooter.
 *
 * RPM and hood interpolation is handled by ShooterSubsystem.updateFromDistance()
 * using the FLYWHEEL_RPM_MAP and HOOD_ROT_MAP tables defined there.
 */
public class VisionShootCommand extends Command {

    // =========================================================================
    // Hub field coordinates (field-relative, meters)
    // =========================================================================
    private static final Translation2d HUB_POSITION = new Translation2d(4.625, 4.025);
    
    // =========================================================================
    // Dependencies
    // =========================================================================
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xSupplier; // driver left-Y scaled to m/s
    private final DoubleSupplier ySupplier; // driver left-X scaled to m/s

    private final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // =========================================================================
    // Diagnostics — published to NT so direction-snap issues can be debugged
    // on the field via Elastic/Shuffleboard.
    // Key: if rotRate sign is opposite to the desired turn direction, negate
    // headingErrorDeg (i.e. swap the subtraction order in headingErrorDeg).
    // =========================================================================
    private final DoublePublisher ntAngleToHub =
            NetworkTableInstance.getDefault()
                    .getTable("VisionShoot").getDoubleTopic("angleToHub_deg").publish();
    private final DoublePublisher ntCurrentHeading =
            NetworkTableInstance.getDefault()
                    .getTable("VisionShoot").getDoubleTopic("currentHeading_deg").publish();
    private final DoublePublisher ntHeadingError =
            NetworkTableInstance.getDefault()
                    .getTable("VisionShoot").getDoubleTopic("headingError_deg").publish();
    private final DoublePublisher ntRotRate =
            NetworkTableInstance.getDefault()
                    .getTable("VisionShoot").getDoubleTopic("rotRate_radps").publish();
    private final DoublePublisher ntDistance =
            NetworkTableInstance.getDefault()
                    .getTable("VisionShoot").getDoubleTopic("distanceToHub_m").publish();

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @param shooter    Shooter subsystem
     * @param indexer    Indexer subsystem
     * @param drivetrain Swerve drivetrain
     * @param xSupplier  Driver left-Y velocity in m/s (already scaled by MaxSpeed)
     * @param ySupplier  Driver left-X velocity in m/s (already scaled by MaxSpeed)
     */
    public VisionShootCommand(
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        addRequirements(shooter, indexer, drivetrain);
    }

    // =========================================================================
    // Command lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        // Spin up immediately on button press using current distance estimate
        Pose2d pose = drivetrain.getState().Pose;
        double distance = getDistanceToHub(pose);
        shooter.updateFromDistance(distance);
        shooter.prepareToShoot();
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getState().Pose;

        // ==== 1. Recalculate distance and push RPM + hood to hardware every loop
        double distance = getDistanceToHub(pose);
        shooter.updateFromDistance(distance);

        if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
            shooter.prepareToShoot();
        }

        // ==== 2. Calculate heading error to hub
        double angleToHubDeg = getAngleToHubDeg(pose);
        double currentHeadingDeg = pose.getRotation().getDegrees();
        double headingErrorDeg = normalizeAngle(angleToHubDeg - currentHeadingDeg);

        // ==== 3. Apply rotation correction, driver controls translation
        double rotRate = MathUtil.clamp(
                headingErrorDeg * Constants.Vision.ROTATIONAL_KP,
                -Constants.Vision.MAX_ROT_RAD_PER_SEC,
                Constants.Vision.MAX_ROT_RAD_PER_SEC);

        // Publish diagnostics — use Elastic/Shuffleboard to verify direction snap:
        // • angleToHub and currentHeading should both be sensible field-frame angles
        // • headingError positive → robot should rotate CCW (left from driver view)
        // • rotRate positive → CCW in CTRE FieldCentric; if robot turns the WRONG way,
        //   negate rotRate and file a note in TUNING.md
        ntAngleToHub.set(angleToHubDeg);
        ntCurrentHeading.set(currentHeadingDeg);
        ntHeadingError.set(headingErrorDeg);
        ntRotRate.set(rotRate);
        ntDistance.set(distance);

        drivetrain.setControl(
                alignRequest
                        .withVelocityX(xSupplier.getAsDouble())
                        .withVelocityY(ySupplier.getAsDouble())
                        .withRotationalRate(rotRate));

        // ==== 4. Feed when aligned — isReady() gate removed
        boolean aligned = Math.abs(headingErrorDeg) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEG;
        if (aligned) {
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
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until trigger is released (whileTrue)
    }

    // =========================================================================
    // Helper methods
    // =========================================================================

    /** Returns the straight-line distance from the robot to the hub in meters. */
    private double getDistanceToHub(Pose2d pose) {
        double dx = HUB_POSITION.getX() - pose.getX();
        double dy = HUB_POSITION.getY() - pose.getY();
        double raw = Math.sqrt(dx * dx + dy * dy);
        return MathUtil.clamp(raw, Constants.Vision.MIN_DISTANCE_M, Constants.Vision.MAX_DISTANCE_M);
    }

    /** Returns the field-relative angle (degrees) the robot needs to face to point at the hub. */
    private double getAngleToHubDeg(Pose2d pose) {
        double dx = HUB_POSITION.getX() - pose.getX();
        double dy = HUB_POSITION.getY() - pose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    /** Normalizes an angle to [-180, 180] degrees. */
    private double normalizeAngle(double deg) {
        while (deg > 180)  deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}