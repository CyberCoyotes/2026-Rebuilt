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
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotPreset;

/**
 * Fuel Commands - Factory for shooter-related commands.
 *
 * This class provides static factory methods to create common shooter commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 * 
 */
public class FuelCommands {

    /**
     * Returns the hub center for the current alliance (defaults to blue if FMS not
     * connected).
     */
    private static Translation2d getHubLocation() {
        return DriverStation.getAlliance()
                .filter(a -> a == DriverStation.Alliance.Red)
                .map(a -> Constants.Vision.RED_HUB_LOCATION)
                .orElse(Constants.Vision.BLUE_HUB_LOCATION);
    }

    private static double getRobotFrontTargetHeadingDegrees(double angleToHubDeg, double aimOffsetDeg) {
        double targetHeadingDeg = angleToHubDeg + aimOffsetDeg + Constants.Vision.ALIGNMENT_OFFSET_DEGREES;
        return MathUtil.inputModulus(targetHeadingDeg, -180.0, 180.0);
    }

    private static double getHeadingErrorDegrees(double targetHeadingDeg, double currentHeadingDeg) {
        return MathUtil.inputModulus(targetHeadingDeg - currentHeadingDeg, -180.0, 180.0);
    }

    // =========================================================================
    // PRIMARY SHOOT COMMANDS
    // =========================================================================

    /**
     * While held: extends intake slides, runs intake purge roller, and reverses the
     * conveyor.
     * On release: stops intake roller/conveyor and retracts the slides.
     *
     * This lives in the shared command factory because it coordinates two
     * subsystems
     * (intake + indexer) and should own both requirements.
     */
    public static Command purgeFuel(IntakeSubsystem intake, IndexerSubsystem indexer) {
        return Commands.runEnd(
                () -> {
                    intake.extendSlidesFast();
                    intake.reverseRoller();
                    ;
                    indexer.reverseConveyor();
                },
                () -> {
                    intake.stopRoller();
                    indexer.conveyorStop();
                    intake.retractSlidesFast();
                },
                intake, indexer)
                .withName("PurgeFuelHeld");
    }

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
                .finallyDo(() -> shooter.setPostShotState())
                .withName("ShootAtCurrentTarget");
    }

    /**
     * Full preset shoot sequence that owns both the shooter and indexer subsystems.
     *
     * Flow:
     * 1. Sets the given RPM + hood target silently
     * 2. Calls beginSpinUp() — flywheel ramps up, hood moves to target
     * 3. Waits until both are at target (isReady()), with a 3-second safety timeout
     * 4. Runs indexer and conveyor forward to feed the game piece
     * 5. On trigger release (whileTrue interrupt): stops indexer/conveyor, returns
     * shooter to standby
     *
     * Use with whileTrue() on the shoot trigger.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param rpm     Target flywheel velocity in RPM
     * @param hood    Target hood position in rotations
     * @return Complete preset shoot command requiring both subsystems
     */
    public static Command shootWithPreset(ShooterSubsystem shooter, IndexerSubsystem indexer,
            double rpm, double hood) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(rpm); // Set Constants._RPM
                    shooter.setTargetHoodPose(hood); // Set Constants._HOOD
                    shooter.beginSpinUp(); // void — transitions state machine to SPINNING_UP
                }, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0), // TODO double check
                Commands.run(() -> {
                    indexer.conveyorForward();
                    indexer.kickerForward();
                }, indexer))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setPostShotState();
                }).withName("ShootWithPreset[" + rpm + "rpm]");
    }

    /**
     * Shoots at the given RPM/hood preset while simultaneously retracting the
     * intake slides (two-phase) and running the intake roller.
     *
     * The shooter+indexer sequence is the deadline — slide retraction runs in
     * parallel and is cancelled when the shot completes or the trigger is released.
     * The intake roller is always stopped on exit via the intake command's
     * finallyDo.
     *
     * Use wherever {@link #shootWithPreset} is used when slides need to retract
     * during the shot (e.g. after intaking from the trench).
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param intake  The intake subsystem
     * @param rpm     Target flywheel velocity in RPM
     * @param hood    Target hood position in rotations
     * @return Deadline command: shoot sequence (deadline) + slide retract in
     *         parallel
     */
    // public static Command shootWithSlideRetract(ShooterSubsystem shooter,
    // IndexerSubsystem indexer,
    // IntakeSubsystem intake, double rpm, double hood) {
    // return Commands.deadline(
    // shootWithPreset(shooter, indexer, rpm, hood),
    // intake.retractSlidesAuton())
    // .withName("ShootWithSlideRetract[" + rpm + "rpm]");
    // }

    public static Command shootPresetAuton(ShooterSubsystem shooter, IndexerSubsystem indexer,
            ShotPreset preset, double feedSeconds) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(preset.rpm);
                    shooter.setTargetHoodPose(preset.hood);
                    shooter.beginSpinUp(); // void — transitions state machine to SPINNING_UP
                }, shooter),
                Commands.waitUntil(shooter::isReady),
                indexer.feed().withTimeout(feedSeconds))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setPostShotState();
                }).withName("ShootPresetAuton[" + preset.label + "]");
    }

    /**
     * Convenience overload — shoots using a {@link ShotPreset} enum value.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param preset  The named preset to use
     * @return Complete preset shoot command
     */
    public static Command shootWithPreset(ShooterSubsystem shooter, IndexerSubsystem indexer,
            ShotPreset preset) {
        return shootWithPreset(shooter, indexer, preset.rpm, preset.hood)
                .withName("ShootWithPreset[" + preset.label + "]");
    }

    /**
     * Air popper assist with intake.
     *
     * While held: runs intake + indexer feed while shooter stays in POPPER preset.
     */
    public static Command runAirPopper(IndexerSubsystem indexer, ShooterSubsystem shooter,
            IntakeSubsystem intake) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setAirPopper(); // enters ShooterState.POPPER — vision cannot override
                }, shooter),
                Commands.deadline(
                        Commands.deadline( // parallel is NOT OK here!
                                intake.intakeFuel(),
                                indexer.feed()),
                        Commands.run(() -> {
                        }, shooter)))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setPostShotState();
                }).withName("RunAirPopperWithIntake");
    }

    // =========================================================================
    // UTILITY COMMANDS
    // =========================================================================

    /**
     * Creates a command to eject jammed game pieces.
     * Reverses flywheel for a duration then returns to SPINUP.
     *
     * @param shooter         The shooter subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects then returns to idle
     */
    public static Command eject(ShooterSubsystem shooter, double durationSeconds) {
        return Commands.sequence(
                Commands.runOnce(shooter::eject, shooter),
                Commands.waitSeconds(durationSeconds)).withName("EjectShooter");
    }

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

    public static Command shootPass(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.pass(); // enters ShooterState.PASS — vision cannot override
                }, shooter),
                Commands.waitUntil(shooter::isPassReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.conveyorForward();
                    indexer.kickerForward();
                }, indexer))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setPostShotState();
                }).withName("ShootPass");
    }

    // =========================================================================
    // POSE ALIGN AND SHOOT (primary match command — replaces VisionShootCommand)
    // =========================================================================
    /**
     * Creates a Command that continuously aligns the swerve drivetrain to the field-relative hub
     * target while allowing limited manual translation input.
     *
     * <p>Behavior:
     * - Computes the 2D vector from the robot pose to the hub (field coordinates) via getHubLocation()
     *   and drivetrain.getState().Pose.
     * - Uses the Euclidean distance (meters) clamped to [Constants.Vision.MIN_DISTANCE_M,
     *   Constants.Vision.MAX_DISTANCE_M] for any downstream diagnostics/logic.
     * - Computes the bearing to the hub in degrees (angleToHubDeg = atan2(dy, dx) in degrees).
     * - Target heading is set to angleToHubDeg + Constants.Vision.ALIGNMENT_OFFSET_DEGREES and
     *   normalized to [-180, 180) degrees.
     * - Heading error is the normalized difference between target heading and the current robot
     *   heading (degrees) and is centered on 0°.
     * - Rotational correction (rotRate):
     *     - If |headingErrorDeg| <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES then rotRate = 0.0
     *       (deadband for settling).
     *     - Otherwise rotRate = headingErrorDeg * Constants.Vision.ROTATIONAL_KP, then clamped to
     *       ±Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC. Note: rotRate is in radians/second.
     * - While the command is active it calls drivetrain.setControl(...) with a FieldCentric
     *   SwerveRequest configured for DriveRequestType.OpenLoopVoltage:
     *     - Velocity X = -xSupplier.getAsDouble() * 0.40
     *     - Velocity Y = -ySupplier.getAsDouble() * 0.40
     *     - Rotational rate = rotRate
     *   The translation inputs are negated and scaled to 40% to limit motion while auto-aiming
     *   (negation compensates for observed backward/forward inversion in vision-assisted mode).
     *
     * <p>Diagnostics and side effects:
     * - Publishes the following keys to NetworkTables (table "VisionShoot"):
     *     - "angleToHub_deg"         : angleToHubDeg (degrees)
     *     - "currentHeading_deg"     : currentHeadingDeg (degrees)
     *     - "headingError_deg"       : headingErrorDeg (degrees)
     *     - "rotRate_radps"          : rotRate (radians per second)
     *     - "distanceToHub_m"        : distance (meters, clamped)
     * - Calls drivetrain.setControl(...) on every execution cycle.
     *
     * <p>Concurrency and lifetime:
     * - The returned Command is non-blocking and runs repeatedly while scheduled. It requires the
     *   provided drivetrain subsystem (is bound to it) and will typically be used as a repeatedly
     *   executed/continuous command (e.g., whileTrue). It does not itself terminate based on
     *   alignment; cancellation or scheduling logic must be handled by the caller.
     *
     * <p>Units and ranges:
     * - Distances are in meters.
     * - Intermediate and published angles are in degrees.
     * - rotRate is in radians per second and is clamped to
     *   ±Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC.
     *
     * @param drivetrain the CommandSwerveDrivetrain used to read pose and issue setControl(...) calls;
     *                   this command will require that subsystem while active
     * @param xSupplier  supplier for the desired X translation input (typically a joystick axis).
     *                   The value is negated and scaled by 0.40 before being sent to the drivetrain.
     * @param ySupplier  supplier for the desired Y translation input (typically a joystick axis).
     *                   The value is negated and scaled by 0.40 before being sent to the drivetrain.
     * @return a Command that, when scheduled, continuously orients the robot toward the field hub
     *         while allowing scaled manual translation and publishing diagnostic values to NetworkTables.
     */
    public static Command poseAlign(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // NT diagnostics — created once per factory call (whileTrue caches the command
        // object)
        final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("VisionShoot");
        final DoublePublisher ntAngleToHub = visionTable.getDoubleTopic("angleToHub_deg").publish();
        final DoublePublisher ntCurrentHeading = visionTable.getDoubleTopic("currentHeading_deg").publish();
        final DoublePublisher ntHeadingError = visionTable.getDoubleTopic("headingError_deg").publish();
        final DoublePublisher ntRotRate = visionTable.getDoubleTopic("rotRate_radps").publish();
        final DoublePublisher ntDistance = visionTable.getDoubleTopic("distanceToHub_m").publish();

        return Commands.run(() -> {
            /**
             * Field-relative 2D translation of the hub target.
             *
             * Retrieved from getHubLocation() and used by fuel-related commands for aiming,
             * trajectory generation, and proximity/interaction checks with the hub.
             */
            Translation2d hub = getHubLocation();
            Pose2d pose = drivetrain.getState().Pose;

            /**
             * Calculates the Euclidean distance from the robot to the target using the horizontal (dx)
             * and vertical (dy) offsets, then constrains that distance to the valid vision range.
             *
             * - Computes the straight-line distance in meters via Math.hypot(dx, dy).
             * - Ensures the returned distance lies within [Constants.Vision.MIN_DISTANCE_M,
             *   Constants.Vision.MAX_DISTANCE_M] to respect sensor and algorithm limits.
             *
             * The final clamped value represents the effective distance (in meters) to be used by
             * downstream logic.
             */

            double dx = hub.getX() - pose.getX();
            double dy = hub.getY() - pose.getY();
            double distance = MathUtil.clamp(
                    Math.hypot(dx, dy),
                    Constants.Vision.MIN_DISTANCE_M,
                    Constants.Vision.MAX_DISTANCE_M);

            double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));

            // Target heading is 180° from hub + lead offset. Error centered around 0°.
            double targetHeadingDeg = MathUtil.inputModulus(angleToHubDeg + Constants.Vision.ALIGNMENT_OFFSET_DEGREES,
                    -180.0, 180.0);
            double currentHeadingDeg = pose.getRotation().getDegrees();
            double headingErrorDeg = MathUtil.inputModulus(targetHeadingDeg - currentHeadingDeg, -180.0, 180.0);

            // Rotation correction — zero output inside deadband so robot settles cleanly
            double rotRate = Math.abs(headingErrorDeg) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES ? 0.0
                    : MathUtil.clamp(
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
                            .withVelocityX(-xSupplier.getAsDouble() * .40) // Limit to 40% while auto-shooting
                            .withVelocityY(-ySupplier.getAsDouble() * .40) // Made negative to correct backwards driving
                                                                           // when vision-assisted
                            .withRotationalRate(rotRate));

        }, drivetrain).withName("PoseAlign"); // End of command

    } // end of poseAlign command

    /**
     * Pose-based hub alignment and shoot command.
     *
     * Replaces VisionShootCommand as a reusable factory method.
     *
     * Behavior (while trigger held):
     * 1. Computes robot-to-hub distance and bearing from drivetrain odometry.
     * 2. Calls updateFromDistance() every loop — keeps RPM and hood live-tracking.
     * 3. Drives rotation toward the point 180° OPPOSITE the hub bearing so the
     * shooter (back of robot) faces the hub. Error is centered around 0° so
     * the deadband (ALIGNMENT_TOLERANCE_DEGREES) works correctly.
     * 4. Feeds indexer once within ALIGNMENT_TOLERANCE_DEGREES AND shooter
     * isReady().
     * 5. On release: stops indexer, conveyor, returns shooter to idle.
     *
     * @param shooter    Shooter subsystem
     * @param indexer    Indexer subsystem
     * @param drivetrain Swerve drivetrain (provides field pose via
     *                   odometry/AprilTag fusion)
     * @param xSupplier  Driver left-Y velocity in m/s (scaled by MaxSpeed)
     * @param ySupplier  Driver left-X velocity in m/s (scaled by MaxSpeed)
     */
    public static Command poseAlignAndShoot(
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // NT diagnostics — created once per factory call (whileTrue caches the command
        // object)
        final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("VisionShoot");
        final DoublePublisher ntAngleToHub = visionTable.getDoubleTopic("angleToHub_deg").publish();
        final DoublePublisher ntCurrentHeading = visionTable.getDoubleTopic("currentHeading_deg").publish();
        final DoublePublisher ntHeadingError = visionTable.getDoubleTopic("headingError_deg").publish();
        final DoublePublisher ntRotRate = visionTable.getDoubleTopic("rotRate_radps").publish();
        final DoublePublisher ntDistance = visionTable.getDoubleTopic("distanceToHub_m").publish();
        final DoublePublisher ntLeadOffset = visionTable.getDoubleTopic("leadOffset_deg").publish();

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

            // 2. Compute target heading — shooter faces AWAY from hub (back of robot toward
            // hub)
            // angleToHubDeg points FROM robot TO hub.
            // Adding 180° gives the direction opposite the hub — that's where the robot
            // front must point.
            // headingError is then (targetHeading - currentHeading), centered around 0°,
            // so the deadband and P-controller both work correctly.
            double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));

            // Velocity lead compensation — offsets aim opposite to lateral movement
            var speeds = drivetrain.getState().Speeds;
            double vx = speeds.vxMetersPerSecond;
            double vy = speeds.vyMetersPerSecond;
            double hubAngleRad = Math.atan2(dy, dx);
            double lateralVelocity = -vx * Math.sin(hubAngleRad) + vy * Math.cos(hubAngleRad);
            double leadOffsetDeg = -lateralVelocity * Constants.Vision.LEAD_COMPENSATION_DEG_PER_MPS;

            // Target heading is 180° from hub + lead offset. Error centered around 0°.
            double targetHeadingDeg = MathUtil.inputModulus(
                    angleToHubDeg + leadOffsetDeg + Constants.Vision.ALIGNMENT_OFFSET_DEGREES, -180.0, 180.0);
            double currentHeadingDeg = pose.getRotation().getDegrees();
            double headingErrorDeg = MathUtil.inputModulus(targetHeadingDeg - currentHeadingDeg, -180.0, 180.0);

            ntLeadOffset.set(leadOffsetDeg);

            // 3. Rotation correction — zero output inside deadband so robot settles cleanly
            double rotRate = Math.abs(headingErrorDeg) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES ? 0.0
                    : MathUtil.clamp(
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
                            .withVelocityX(-xSupplier.getAsDouble() * .40) // Limit to 40% while auto-shooting
                            .withVelocityY(-ySupplier.getAsDouble() * .40) // Made negative to correct backwards driving
                                                                           // when vision-assisted
                            .withRotationalRate(rotRate));

            // 4. Feed: aligned within ALIGNMENT_TOLERANCE_DEGREES AND flywheel/hood settled
            if (shooter.isReady() && Math.abs(headingErrorDeg) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES) {
                indexer.conveyorForward();
                indexer.kickerForward();
            } else {
                indexer.indexerStop();
                indexer.conveyorStop();
            }

        }, shooter, indexer, drivetrain /* , intake */)
                .beforeStarting(Commands.runOnce(() -> {
                    shooter.beginSpinUp();
                }, shooter))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setPostShotState();
                })
                .withName("PoseAlignAndShoot");
    }

    public static class Auto {

        // ============================================================================
        // RobotContainer.java — add to configureNamedCommands() or wherever you
        // register Choreo event markers. This wires the "Shoot" event marker in your
        // .chor file to the new command.
        // ============================================================================

        /*
         * In RobotContainer.java — inside configureNamedCommands() or the
         * constructor:
         * 
         * NamedCommands.registerCommand("Shoot",
         * ShooterCommands.visionAlignAndShoot(shooter, vision, indexer, drivetrain));
         * 
         * If you want the shooter pre-warming while Choreo drives to position,
         * also register a "SpinUp" marker to fire earlier in the path:
         * NamedCommands.registerCommand("SpinUp",
         * ShooterCommands.visionShot(shooter, vision)); // start warming early
         */

        // ============================================================================
        // TUNING NOTES FOR STUDENTS
        // ============================================================================
        /*
         * The two numbers students are most likely to need to adjust:
         * 
         * 1. alignPID kP (currently 0.05):
         * - Too low → robot barely turns, takes forever to align
         * - Too high → robot oscillates back and forth around the target
         * - Start at 0.05, increase slowly until it snaps to target quickly without
         * overshoot
         * 
         * 2. Max rotation clamp (currently 1.0 rad/s):
         * - Reduces this if the robot overshoots badly during auto
         * - Try 0.5 rad/s as a conservative starting point
         * 
         * 3. feedTimed duration (currently 0.5 seconds):
         * - Increase if game pieces aren't making it all the way through
         * - Decrease if you're wasting time waiting after the piece is gone
         * 
         * HOW TO VERIFY IT'S WORKING:
         * - Watch the Vision/IsAligned signal on Elastic during auto replay
         * - In AdvantageScope: plot "Vision/HorizontalAngle_deg" — should approach 0
         * before feed fires
         * - If Phase 1 times out at 3s, the robot still proceeds to feed (by design for
         * now)
         * If you want to SKIP the shot on timeout, wrap the sequence in:
         * .onlyIf(() -> vision.hasTarget())
         */

        // =============================================================================
        /**
         * Autonomous odometry-based align-and-shoot.
         *
         * Uses drivetrain odometry to compute heading error to the hub — no vision,
         * no driver input, no lead compensation (robot is stopped).
         *
         * Phase 1 (deadline): rotate toward hub + spin up shooter simultaneously.
         * Ends when heading error ≤ ALIGNMENT_TOLERANCE_DEGREES AND shooter isReady(),
         * or after a 3-second safety timeout.
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
         * @param shotTimeout Timeout for the shooting sequence
         * @return Autonomous align-and-shoot command
         */
        public static Command poseAlignAndShoot(
                ShooterSubsystem shooter,
                IndexerSubsystem indexer,
                IntakeSubsystem intake,
                CommandSwerveDrivetrain drivetrain,
                double shotTimeout) {

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
                                double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));
                                double targetHeadingDeg = getRobotFrontTargetHeadingDegrees(angleToHubDeg, 0.0);
                                double headingErrorDeg = getHeadingErrorDegrees(
                                        targetHeadingDeg,
                                        pose.getRotation().getDegrees());
                                return Math.abs(headingErrorDeg) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES
                                        && shooter.isReady();
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
                                double angleToHubDeg = Math.toDegrees(Math.atan2(dy, dx));
                                double targetHeadingDeg = getRobotFrontTargetHeadingDegrees(angleToHubDeg, 0.0);
                                double headingErrorDeg = getHeadingErrorDegrees(
                                        targetHeadingDeg,
                                        pose.getRotation().getDegrees());
                                double rotRate = MathUtil.clamp(
                                        headingErrorDeg * Constants.Vision.ROTATIONAL_KP,
                                        -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                                        Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);
                                drivetrain.setControl(alignRequest
                                        .withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(rotRate));
                            }, shooter, drivetrain)),
                    // Phase 2: feed until chute clears while compressing fuel once shooter is ready
                    Commands.deadline(
                            indexer.feedUntilChuteEmpty(shotTimeout),
                            fuelCompressionWhenShooterReady(shooter, intake)))
                    .finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setPostShotState();
                    }).withName("Auto.PoseAlignAndShoot");
        }

        private static Command fuelCompressionWhenShooterReady(
                ShooterSubsystem shooter,
                IntakeSubsystem intake) {
            return Commands.sequence(
                    Commands.waitUntil(shooter::isReady),
                    intake.fuelCompression())
                    .withName("FuelCompressionWhenShooterReady");
        }

        // =============================================================================
        // Presets
        // =============================================================================
        public static Command shootTrench(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double shotTimeout) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Flywheel.TRENCH_RPM);
                        shooter.setTargetHoodPose(Constants.Hood.TRENCH_HOOD);
                        shooter.beginSpinUp();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady),
                    indexer.feedUntilChuteEmpty(shotTimeout)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setPostShotState();
                    }).withName("ShootTrenchAuton");
        }

        public static Command shootHub(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double shotTimeout) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Flywheel.CLOSE_RPM);
                        shooter.setTargetHoodPose(Constants.Hood.CLOSE_HOOD);
                        shooter.beginSpinUp();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady),
                    indexer.feedUntilChuteEmpty(shotTimeout)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setPostShotState();
                    }).withName("ShootHubAuton");
        }

        public static Command shootTower(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double shotTimeout) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Flywheel.TOWER_RPM);
                        shooter.setTargetHoodPose(Constants.Hood.TOWER_HOOD);
                        shooter.beginSpinUp();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feedUntilChuteEmpty(shotTimeout)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setPostShotState();
                    }).withName("ShootTowerAuton");
        } // end of command

        public static Command shootFar(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double shotTimeout) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Flywheel.FAR_RPM);
                        shooter.setTargetHoodPose(Constants.Hood.FAR_HOOD);
                        shooter.beginSpinUp();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feedUntilChuteEmpty(shotTimeout)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setPostShotState();
                    }).withName("ShootFarAuton");
        } // end of command

        /**
         * Sensor-gated fuel pump for autonomous — bridges IntakeSubsystem and
         * IndexerSubsystem.
         *
         * Phase 1 — WAIT (no subsystem held): polls {@code indexer.isFuelDetected()}.
         * While waiting, IntakeSubsystem is free so an {@code intakeFuelTimer} can run
         * in parallel without conflict.
         *
         * Phase 2 — PUMP (claims IntakeSubsystem): "bounces" slides + runs roller
         * until {@code indexer.isChuteEmpty()} is true (fuel seen then gone for
         * FUEL_CLEAR_TIME seconds).
         *
         * A hard-stop timeout prevents the command from hanging if the sensor lies
         * or fuel never clears.
         *
         * @param intake  The IntakeSubsystem that owns the slides and roller.
         * @param indexer The IndexerSubsystem that owns the chute CANrange sensor.
         */
        public static Command fuelPumpCycleSensor(IntakeSubsystem intake, IndexerSubsystem indexer) {
            return Commands.sequence(
                    // Phase 1: wait for first fuel — no subsystem required, intake can run freely
                    Commands.runOnce(indexer::resetChuteTracking),
                    Commands.waitUntil(indexer::isFuelDetected),
                    // Phase 2: pump until chute clears (or hard timeout)
                    intake.fuelPumpCycleUntil(
                            indexer::isChuteEmpty,
                            Constants.Intake.SLIDE_FUEL_PUMP_WAIT_SECONDS,
                            Constants.Intake.SLIDE_FUEL_PUMP_SENSOR_TIMEOUT_SECONDS))
                    .withName("FuelPumpCycleSensor");
        }

    }// end of class Auto

} // end of class FuelCommands

// =======================================================================================
// DEPRECATED/EXPERIMENTAL COMMANDS — use with caution and test thoroughly if
// re-enabling
// =======================================================================================
