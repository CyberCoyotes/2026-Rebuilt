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
 * AutoHubShootCommand — Full auto hub shot command for the driver X button.
 *
 * BEHAVIOR (while held):
 *   1. Spins up the shooter using distance-interpolated RPM + hood from the
 *      robot's current pose relative to the hub center.
 *   2. Rotates the drivetrain toward the hub using a PID controller on
 *      VisionSubsystem.getAngleToHub() (pose-derived, not raw tx).
 *   3. Allows the driver to translate freely with the left stick.
 *   4. Feeds the indexer and conveyor only when BOTH conditions are true:
 *        - Shooter reports isReady()
 *        - Angle error is within FEED_TOLERANCE_DEGREES
 *   5. On button release: stops indexer, conveyor, and shooter.
 *
 * TUNING:
 *   - ROT_kP / ROT_kD: Rotation PID gains
 *   - ROT_MAX_OUTPUT: Caps rotation rate in rad/s
 *   - FEED_TOLERANCE_DEGREES: How tightly aligned before feeding
 *   - MAX_TRACKING_SPEED: Caps driver translation speed during the shot
 */
public class AutoHubShootCommand extends Command {

    // ===== Tuning =====
    private static final double ROT_kP               = 0.15;  // TODO: Tune
    private static final double ROT_kI               = 0.00;
    private static final double ROT_kD               = 0.00;
    private static final double ROT_MAX_OUTPUT        = 3.0;   // rad/s
    private static final double FEED_TOLERANCE_DEGREES = 1.5;  // TODO: Tune
    private static final double MAX_TRACKING_SPEED    = 1.5;   // m/s

    // ===== Subsystems =====
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem         vision;
    private final ShooterSubsystem        shooter;
    private final IndexerSubsystem        indexer;
    private final DoubleSupplier          vx;
    private final DoubleSupplier          vy;

    // ===== Control =====
    private final PIDController              pid;
    private final SwerveRequest.FieldCentric request;

    public AutoHubShootCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            DoubleSupplier vx,
            DoubleSupplier vy) {

        this.drivetrain = drivetrain;
        this.vision     = vision;
        this.shooter    = shooter;
        this.indexer    = indexer;
        this.vx         = vx;
        this.vy         = vy;

        pid = new PIDController(ROT_kP, ROT_kI, ROT_kD);
        pid.setSetpoint(0.0);
        pid.setTolerance(FEED_TOLERANCE_DEGREES);
        pid.enableContinuousInput(-180.0, 180.0);

        request = new SwerveRequest.FieldCentric()
            .withDeadband(0.0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, shooter, indexer);
        // Vision is not added as a requirement — it's read-only here
    }

    @Override
    public void initialize() {
        pid.reset();
        shooter.prepareToShoot();
    }

    @Override
    public void execute() {
        // ── 1. Shooter: update RPM + hood from pose-based distance ───────────
        double dist = vision.getDistanceToHub();
        if (dist > 0) {
            shooter.updateFromDistance(dist);
        }
        if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
            shooter.prepareToShoot();
        }

        // ── 2. Drivetrain: driver translation + pose-based rotation ──────────
        double angleErrorDeg = vision.getAngleToHub();
        double rot = pid.calculate(angleErrorDeg);
        rot = Math.max(-ROT_MAX_OUTPUT, Math.min(ROT_MAX_OUTPUT, rot));

        drivetrain.setControl(
            request
                .withVelocityX(cap(vx.getAsDouble()))
                .withVelocityY(cap(vy.getAsDouble()))
                .withRotationalRate(rot)
        );

        // ── 3. Feed: only when aligned AND shooter is ready ───────────────────
        if (shooter.isReady() && Math.abs(angleErrorDeg) < FEED_TOLERANCE_DEGREES) {
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
        pid.reset();
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button is released (whileTrue)
    }

    private double cap(double v) {
        return Math.max(-MAX_TRACKING_SPEED, Math.min(MAX_TRACKING_SPEED, v));
    }
}