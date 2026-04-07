package frc.robot.visionLegacy;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotPreset;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Fuel Commands - Factory for shooter-related commands.
 *
 * This class provides static factory methods to create common shooter commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 * 
 */
public class FuelCommandsMARCH {

    /** Returns the hub center for the current alliance (defaults to blue if FMS not connected). */
    private static Translation2d getHubLocation() {
        return DriverStation.getAlliance()
                .filter(a -> a == DriverStation.Alliance.Red)
                .map(a -> Constants.Vision.RED_HUB_LOCATION)
                .orElse(Constants.Vision.BLUE_HUB_LOCATION);
    }

    // =========================================================================
    // PRIMARY SHOOT COMMANDS
    // =========================================================================

    /**
     * Shoots at whatever preset is currently selected.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @return Command that shoots at current targets and returns to idle on release
     */

    public static Command shootAtCurrentTarget(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.runOnce(shooter::beginSpinUp, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                indexer.feed())
                .finallyDo(() -> shooter.setIdle())
                .withName("ShootAtCurrentTarget");
    }

    // =========================================================================
    // UTILITY COMMANDS
    // =========================================================================

  

    /**
     * Creates a command that waits until shooter is ready.
     *
     * @param shooter The shooter subsystem
     * @return Command that waits for shooter ready state
     */
    public static Command waitUntilReady(ShooterSubsystem shooter) {
        return Commands.waitUntil(shooter::isReady)
                .withTimeout(3.0)
                .withName("WaitForShooterReady");
    }

    // =========================================================================
    // POSE ALIGN AND SHOOT (primary match command — replaces VisionShootCommand)
    // =========================================================================

    /**
     * Pose-based hub alignment and shoot command.
     *
     * Replaces VisionShootCommand as a reusable factory method.
     *
     * Behavior (while trigger held):
     * 1. Computes robot-to-hub distance and bearing from drivetrain odometry.
     * 2. Calls updateFromDistance() every loop — keeps RPM and hood live-tracking.
     * 3. Drives rotation toward hub via a P-controller; driver controls translation.
     * 4. Feeds indexer once aligned within ALIGNMENT_TOLERANCE_DEGREES AND shooter isReady().
     * 5. On release: stops indexer, conveyor, returns shooter to idle.
     *
     * Feed tolerance fix: uses ALIGNMENT_TOLERANCE_DEGREES (2.0°) not ALIGNMENT_TOLERANCE_DEG
     * (0.5°). A P-only controller always has steady-state error; 0.5° was unreachable.
     *
     * @param shooter    Shooter subsystem
     * @param indexer    Indexer subsystem
     * @param drivetrain Swerve drivetrain (provides field pose via odometry/AprilTag fusion)
     * @param xSupplier  Driver left-Y velocity in m/s (scaled by MaxSpeed)
     * @param ySupplier  Driver left-X velocity in m/s (scaled by MaxSpeed)
     */
    public static Command poseAlignAndShoot(
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            // IntakeSubsystem intake,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // NT diagnostics — created once per factory call (whileTrue caches the command object)
        final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("VisionShoot");
        final DoublePublisher ntAngleToHub     = visionTable.getDoubleTopic("angleToHub_deg").publish();
        final DoublePublisher ntCurrentHeading = visionTable.getDoubleTopic("currentHeading_deg").publish();
        final DoublePublisher ntHeadingError   = visionTable.getDoubleTopic("headingError_deg").publish();
        final DoublePublisher ntRotRate        = visionTable.getDoubleTopic("rotRate_radps").publish();
        final DoublePublisher ntDistance       = visionTable.getDoubleTopic("distanceToHub_m").publish();
        final DoublePublisher ntLeadOffset     = visionTable.getDoubleTopic("leadOffset_deg").publish();

        // final Timer fuelPumpTimer = new Timer();

        return Commands.run(() -> {
            Translation2d hub = getHubLocation();
            Pose2d pose = drivetrain.getState().Pose;

            // 1. Distance → update shooter targets live
            double dx = hub.getX() - pose.getX();
            double dy = hub.getY() - pose.getY();
            double distance = MathUtil.clamp(
                    Math.hypot(dx, dy),
                    Constants.Vision.MIN_DISTANCE_M,
                    Constants.Vision.MAX_DISTANCE_M);

            shooter.updateFromDistance(distance);
            if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
                shooter.beginSpinUp(); // void — only transitions state; never call spinUp() (returns Command) here
            }

            // 2. Apply velocity offset for movement
            double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));

            // Velocity lead compensation — offsets aim opposite to lateral movement
            var speeds = drivetrain.getState().Speeds;
            double vx = speeds.vxMetersPerSecond;
            double vy = speeds.vyMetersPerSecond;
            double hubAngleRad = Math.atan2(dy, dx);
            double lateralVelocity = -vx * Math.sin(hubAngleRad) + vy * Math.cos(hubAngleRad);
            double leadOffsetDeg = -lateralVelocity * Constants.Vision.LEAD_COMPENSATION_DEG_PER_MPS;

            double currentHeadingDeg = pose.getRotation().getDegrees();
            double headingErrorDeg   = angleToHubDeg + leadOffsetDeg - currentHeadingDeg;
            while (headingErrorDeg >  180) headingErrorDeg -= 360;
            while (headingErrorDeg < -180) headingErrorDeg += 360;

            ntLeadOffset.set(leadOffsetDeg);

            // 3. Rotation correction
            double rotRate = MathUtil.clamp(
                    headingErrorDeg * Constants.Vision.ROTATIONAL_KP,
                    -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                    Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);

            ntAngleToHub.set(angleToHubDeg);
            ntCurrentHeading.set(currentHeadingDeg);
            ntHeadingError.set(headingErrorDeg);
            ntRotRate.set(rotRate);
            ntDistance.set(distance);

            drivetrain.setControl(
                    alignRequest
                            .withVelocityX(xSupplier.getAsDouble() *.40) //Setting max drive speed to 40% when Auto Shooting
                            .withVelocityY(ySupplier.getAsDouble() *.40)
                            .withRotationalRate(rotRate));

            // 4. Feed: aligned within 2° AND flywheel/hood settled
            // if (shooter.isReady()) {
            //     indexer.conveyorForward();
            //     indexer.indexerForward();
            // } else {
            //     indexer.indexerStop();
            //     indexer.conveyorStop();
            //     // fuelPumpTimer.reset(); // reset so pump starts fresh when shooter becomes ready
            // }

        }, shooter, indexer, drivetrain /*, intake*/)
                .beforeStarting(Commands.runOnce(() -> {
                    shooter.beginSpinUp();
                    // fuelPumpTimer.reset();
                    // fuelPumpTimer.start();
                }, shooter))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                })
                .withName("PoseAlignAndShoot");
    }

    public static class Auto {


    // =========================================================================



        // =============================================================================
        /**
         * Autonomous odometry-based align-and-shoot.
         *
         * Uses drivetrain odometry to compute heading error to the hub — no vision,
         * no driver input, no lead compensation (robot is stopped).
         *
         * Phase 1 (deadline): rotate toward hub + spin up shooter simultaneously.
         *   Ends when heading error ≤ ALIGNMENT_TOLERANCE_DEGREES AND shooter isReady(),
         *   or after a 3-second safety timeout.
         * Phase 2: feed indexer for feedSeconds.
         * finallyDo: stop indexer/conveyor, return shooter to idle.
         *
         * Use at Choreo "Shoot" event markers. Because this command requires the
         * drivetrain, it will interrupt (take over from) the Choreo path command
         * the moment it is scheduled — place the event marker at the end of the
         * segment or after the robot has reached its shooting position.
         *
         * @param shooter     Shooter subsystem
         * @param indexer     Indexer subsystem
         * @param drivetrain  Drivetrain (odometry pose source)
         * @param feedSeconds How long to run the indexer/conveyor after alignment
         * @return Autonomous align-and-shoot command
         */
        public static Command poseAlignAndShoot(
                ShooterSubsystem shooter,
                IndexerSubsystem indexer,
                CommandSwerveDrivetrain drivetrain,
                double safetyTimeout) {

            final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            return Commands.sequence(
                    // Phase 1: rotate to hub + spin up — both must be ready before feeding
                    Commands.deadline(
                            Commands.waitUntil(() -> {
                                Translation2d hub = getHubLocation();
                                Pose2d pose = drivetrain.getState().Pose;
                                double dx = hub.getX() - pose.getX();
                                double dy = hub.getY() - pose.getY();
                                double headingErrorDeg = Math.toDegrees(Math.atan2(dy, dx))
                                        - pose.getRotation().getDegrees();
                                while (headingErrorDeg >  180) headingErrorDeg -= 360;
                                while (headingErrorDeg < -180) headingErrorDeg += 360;
                                return Math.abs(headingErrorDeg) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES
                                        && shooter.isReady();
                            /* Added .withTimeout(3.0) and working now. Post weekend, explore why timeout is needed
                             * Lower to 1.0 and see if it still works — if not, we know the issue is that the command is waiting indefinitely for the alignment condition to be met, which never happens because of some bug in the alignment code. The timeout allows it to move on to feeding even if the alignment condition is never satisfied, which is better than doing nothing at all.
                            */
                                    }).withTimeout(1.0), 
                            
                            Commands.run(() -> {
                                Translation2d hub = getHubLocation();
                                Pose2d pose = drivetrain.getState().Pose;
                                double dx = hub.getX() - pose.getX();
                                double dy = hub.getY() - pose.getY();
                                double distance = MathUtil.clamp(
                                        Math.hypot(dx, dy),
                                        Constants.Vision.MIN_DISTANCE_M,
                                        Constants.Vision.MAX_DISTANCE_M);
                                shooter.updateFromDistance(distance);
                                if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
                                    shooter.beginSpinUp();
                                }
                                double headingErrorDeg = Math.toDegrees(Math.atan2(dy, dx))
                                        - pose.getRotation().getDegrees();
                                while (headingErrorDeg >  180) headingErrorDeg -= 360;
                                while (headingErrorDeg < -180) headingErrorDeg += 360;
                                double rotRate = MathUtil.clamp(
                                        headingErrorDeg * Constants.Vision.ROTATIONAL_KP,
                                        -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                                        Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);
                                drivetrain.setControl(alignRequest
                                        .withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(rotRate));
                            }, shooter, drivetrain)),
                    // Phase 2: feed until chute clears (safetyTimeout is the fallback)
                    indexer.feedUntilChuteEmpty(safetyTimeout)
            ).finallyDo(() -> {
                indexer.indexerStop();
                indexer.conveyorStop();
                shooter.setIdle();
            }).withName("Auto.PoseAlignAndShoot");
        }

    } 
    // end of class Auto

} // end of class FuelCommands