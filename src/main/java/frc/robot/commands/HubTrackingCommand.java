package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * HubTrackingCommand
 *
 * Runs while the shoot button is HELD (whileTrue) when hub shot mode is armed (X button).
 * Responsibilities:
 *   - Rotates the robot to face the hub using field-relative pose angle (getAngleToHub)
 *   - Updates flywheel RPM and hood angle continuously from live distance
 *   - Caps translation speed so the driver can still strafe but can't move too fast
 *   - Releases drivetrain and returns shooter to idle when button is released
 *
 * The shooter state machine handles isReady() internally.
 * Indexer feeding is triggered by a separate whileTrue binding in RobotContainer
 * that checks shooter.isReady() — keeping feeding logic out of this command.
 *
 * WIRING (in RobotContainer):
 *
 *   // X: Arm hub shot preset (spins up flywheel at default hub RPM)
 *   driver.x().onTrue(new InstantCommand(shooter::hubShot, shooter));
 *
 *   Trigger hubShotArmed = new Trigger(shooter::isHubShotArmed);
 *
 *   // RT (hub shot armed): Track hub and update shot parameters while held
 *   driver.rightTrigger(0.5).and(hubShotArmed).whileTrue(
 *       new HubTrackingCommand(drivetrain, shooter, vision,
 *           () -> -driver.getLeftY() * MaxSpeed,
 *           () -> -driver.getLeftX() * MaxSpeed)
 *   );
 *
 *   // RT (hub shot armed): Feed indexer while shooter is ready
 *   driver.rightTrigger(0.5).and(hubShotArmed).and(shooter::isReady).whileTrue(
 *       ShooterCommands.feedOnly(indexer)
 *   );
 *
 *   // RT (standard shot): Close or pass shot at currently armed preset
 *   driver.rightTrigger(0.5).and(hubShotArmed.negate()).whileTrue(
 *       ShooterCommands.shootAtCurrentTarget(shooter, indexer)
 *   );
 */
public class HubTrackingCommand extends Command {

    // -------------------------------------------------------------------------
    // Rotation PID — tuned for tx in degrees, output in rad/s for swerve
    // -------------------------------------------------------------------------

    private static final double ROT_kP         = 0.10;   // TODO: Tune
    private static final double ROT_kI         = 0.00;
    private static final double ROT_kD         = 0.003;  // TODO: Tune
    private static final double ROT_MIN_OUTPUT = 0.05;    // Minimum nudge to overcome static friction
    private static final double ROT_MAX_OUTPUT = 3.5;    // Max rotation rate (rad/s)

    private static final double ALIGN_TOLERANCE_DEG = 1.5; // TODO: Tune — tighter once PID is dialed

    // -------------------------------------------------------------------------
    // Translation speed cap while tracking
    // -------------------------------------------------------------------------

    /** Max translation speed while hub tracking is active (m/s).
     *  Lower than normal MaxSpeed so the driver can still reposition
     *  but can't blast across the field while shooting. */
    private static final double MAX_TRACKING_SPEED_MPS = 1.5; // TODO: Tune

    // -------------------------------------------------------------------------
    // Angle smoothing
    // -------------------------------------------------------------------------

    private static final int ANGLE_FILTER_TAPS = 4;

    // -------------------------------------------------------------------------
    // Dependencies
    // -------------------------------------------------------------------------

    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem        shooter;
    private final VisionSubsystem         vision;

    /** Supplies the driver's requested X velocity (m/s), already scaled by MaxSpeed.
     *  Pass in () -> -driver.getLeftY() * MaxSpeed from RobotContainer. */
    private final java.util.function.DoubleSupplier velocityXSupplier;

    /** Supplies the driver's requested Y velocity (m/s), already scaled by MaxSpeed.
     *  Pass in () -> -driver.getLeftX() * MaxSpeed from RobotContainer. */
    private final java.util.function.DoubleSupplier velocityYSupplier;

    // -------------------------------------------------------------------------
    // Swerve request — FieldCentric so translation stays driver-relative
    // -------------------------------------------------------------------------

    private final SwerveRequest.FieldCentric trackRequest;

    // -------------------------------------------------------------------------
    // Control objects
    // -------------------------------------------------------------------------

    private final PIDController rotationPID;
    private final LinearFilter  angleFilter;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------

    private double smoothedAngle;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * @param drivetrain        CTRE swerve drivetrain
     * @param shooter           Shooter subsystem — for live RPM/hood updates
     * @param vision            VisionSubsystem — for getAngleToHub() and getDistanceToHub()
     * @param velocityXSupplier Driver's requested X velocity in m/s (e.g. () -> -driver.getLeftY() * MaxSpeed)
     * @param velocityYSupplier Driver's requested Y velocity in m/s (e.g. () -> -driver.getLeftX() * MaxSpeed)
     */
    public HubTrackingCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            java.util.function.DoubleSupplier velocityXSupplier,
            java.util.function.DoubleSupplier velocityYSupplier) {

        this.drivetrain        = drivetrain;
        this.shooter           = shooter;
        this.vision            = vision;
        this.velocityXSupplier = velocityXSupplier;
        this.velocityYSupplier = velocityYSupplier;

        rotationPID = new PIDController(ROT_kP, ROT_kI, ROT_kD);
        rotationPID.setSetpoint(0.0);
        rotationPID.setTolerance(ALIGN_TOLERANCE_DEG);
        // No enableContinuousInput — getAngleToHub() already returns a wrapped delta angle

        angleFilter = LinearFilter.movingAverage(ANGLE_FILTER_TAPS);

        trackRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Does NOT require indexer — feeding is a separate binding
        addRequirements(drivetrain, shooter);
    }

    // -------------------------------------------------------------------------
    // Command lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void initialize() {
        rotationPID.reset();
        angleFilter.reset();
        smoothedAngle = 0.0;

        // Arm the hub preset and spin up flywheels immediately.
        // updateFromDistance() will override these every tick once we have a valid pose.
        shooter.setHubShotPreset();
        shooter.prepareToShoot();

        // Seed filter with real angle so we don't snap from zero on first tick
        smoothedAngle = angleFilter.calculate(vision.getAngleToHub());
    }

    @Override
    public void execute() {
        // ------------------------------------------------------------------
        // 1. Update shooter targets from live distance
        // ------------------------------------------------------------------
        double dist = vision.getDistanceToHub();
        if (dist > 0) {
            shooter.updateFromDistance(dist);
        }

        // ------------------------------------------------------------------
        // 2. Compute rotation output from pose-based hub angle
        // ------------------------------------------------------------------
        smoothedAngle = angleFilter.calculate(vision.getAngleToHub());

        double rotOutput = 0.0;
        if (Math.abs(smoothedAngle) > ALIGN_TOLERANCE_DEG) {
            double raw = rotationPID.calculate(smoothedAngle);
            // Enforce minimum output so we always overcome static friction
            raw = Math.abs(raw) < ROT_MIN_OUTPUT
                    ? Math.copySign(ROT_MIN_OUTPUT, raw)
                    : raw;
            rotOutput = Math.max(-ROT_MAX_OUTPUT, Math.min(ROT_MAX_OUTPUT, raw));
        }

        // ------------------------------------------------------------------
        // 3. Apply field-centric request with translation cap and hub rotation
        //    Translation values come in externally via the default command,
        //    but we override them here with a capped version.
        //    We read stick values directly so the driver keeps full control
        //    of WHERE to go, just at a reduced speed.
        // ------------------------------------------------------------------
        drivetrain.setControl(
            trackRequest
                .withVelocityX(getCapTranslation(velocityXSupplier.getAsDouble()))
                .withVelocityY(getCapTranslation(velocityYSupplier.getAsDouble()))
                .withRotationalRate(rotOutput)
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooter.returnToIdle();
        rotationPID.reset();
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        // Runs until button is released — WPILib handles cancellation via whileTrue
        return false;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /** Clamps a translation velocity to MAX_TRACKING_SPEED_MPS. */
    private double getCapTranslation(double velocity) {
        return Math.max(-MAX_TRACKING_SPEED_MPS, Math.min(MAX_TRACKING_SPEED_MPS, velocity));
    }
}