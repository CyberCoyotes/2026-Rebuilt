package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Set;

/**
 * AutoAlignAndShootCommand - Aligns to hub AprilTag and shoots in autonomous.
 *
 * PHASES:
 * 1. ALIGNING — rotates to hub, spins up flywheel and positions hood.
 *               updateFlywheelVelocity() and updateHoodForDistance() re-command
 *               motors every loop tick so the flywheel actually reaches target.
 *               Transitions to FEEDING when shooter.isReady() OR after 3 seconds.
 * 2. FEEDING  — brakes drivetrain, runs indexer and conveyor for shootDurationSeconds.
 */
public class AutoAlignAndShootCommand extends Command {

    private enum Phase { ALIGNING, FEEDING }

    private static final Set<Integer> HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    private static final double CLOSE_DISTANCE_METERS = 0.85;
    private static final double FAR_DISTANCE_METERS = 3.5;
    private static final double MIN_RPM = 2875;
    private static final double MAX_RPM = 3500;
    private static final double TX_FILTER_ALPHA = 0.2;

    private static final double kP = 0.05;      // TODO: Tune
    private static final double kI = 0.0;
    private static final double kD = 0.001;
    private static final double MIN_ROTATION_OUTPUT = 0.5;   // TODO: Tune
    private static final double MAX_ROTATION_RATE = 4.0;      // TODO: Tune
    private static final double ALIGN_TOLERANCE_DEGREES = 3.0;
    private static final double MAX_ALIGN_WAIT_SECONDS = 1.0; // TODO: Tune

    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;
    private final double shootDurationSeconds;

    private final SwerveRequest.FieldCentric alignRequest;
    private final SwerveRequest.SwerveDriveBrake brakeRequest;
    private final PIDController rotationPID;

    private double smoothedTx = 0.0;
    private Phase currentPhase = Phase.ALIGNING;
    private double feedingElapsedSeconds = 0.0;
    private double aligningElapsedSeconds = 0.0;

    public AutoAlignAndShootCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            IndexerSubsystem indexer,
            double shootDurationSeconds) {

        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.vision = vision;
        this.indexer = indexer;
        this.shootDurationSeconds = shootDurationSeconds;

        rotationPID = new PIDController(kP, kI, kD);
        rotationPID.setSetpoint(0.0);
        rotationPID.setTolerance(ALIGN_TOLERANCE_DEGREES);
        rotationPID.enableContinuousInput(-180.0, 180.0);

        alignRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        brakeRequest = new SwerveRequest.SwerveDriveBrake();

        addRequirements(drivetrain, shooter, vision, indexer);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        currentPhase = Phase.ALIGNING;
        feedingElapsedSeconds = 0.0;
        aligningElapsedSeconds = 0.0;

        // Set targets based on current vision
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            smoothedTx = vision.getHorizontalAngleDegrees();
            double distance = vision.getDistanceToTargetMeters();
            shooter.setTargetVelocity(interpolateRPM(distance));
            shooter.setTargetHoodPose(interpolateHoodAngle(distance));
        } else {
            smoothedTx = 0.0;
            shooter.setTargetVelocity(MIN_RPM);
            shooter.setTargetHoodPose(ShooterSubsystem.CLOSE_SHOT_HOOD);
        }

        // Force motors to receive targets immediately, bypassing setState() guard
        shooter.refreshReady();
    }

    @Override
    public void execute() {
        switch (currentPhase) {

            case ALIGNING: {
                aligningElapsedSeconds += 0.02;

                // Update smoothed tx
                if (vision.hasTarget() && isHubTag(vision.getTagId())) {
                    double rawTx = vision.getHorizontalAngleDegrees();
                    smoothedTx = (TX_FILTER_ALPHA * rawTx) + ((1.0 - TX_FILTER_ALPHA) * smoothedTx);
                }

                // Update flywheel AND hood from live distance every tick —
                // these both re-command the motor immediately in READY state
                if (vision.hasTarget() && isHubTag(vision.getTagId())) {
                    double distance = vision.getDistanceToTargetMeters();
                    shooter.updateFlywheelVelocity(interpolateRPM(distance));
                    shooter.updateHoodForDistance(interpolateHoodAngle(distance));
                }

                // Rotation PID
                double rotationOutput = 0.0;
                if (vision.hasTarget() && isHubTag(vision.getTagId())) {
                    if (Math.abs(smoothedTx) > ALIGN_TOLERANCE_DEGREES) {
                        double pidOutput = rotationPID.calculate(smoothedTx);
                        if (Math.abs(pidOutput) < MIN_ROTATION_OUTPUT) {
                            pidOutput = Math.copySign(MIN_ROTATION_OUTPUT, pidOutput);
                        }
                        rotationOutput = Math.max(-MAX_ROTATION_RATE,
                                         Math.min(MAX_ROTATION_RATE, pidOutput));
                    }
                }

                // Direct setControl — X/Y locked to 0, rotation only
                drivetrain.setControl(
                    alignRequest
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withRotationalRate(rotationOutput)
                );

                indexer.indexerStop();
                indexer.conveyorStop();

                if (shooter.isReady() || aligningElapsedSeconds >= MAX_ALIGN_WAIT_SECONDS) {
                    currentPhase = Phase.FEEDING;
                }
                break;
            }

            case FEEDING: {
                drivetrain.setControl(brakeRequest);
                indexer.indexerForward();
                indexer.conveyorForward();
                feedingElapsedSeconds += 0.02;
                break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.indexerStop();
        indexer.conveyorStop();
        shooter.returnToIdle();
        rotationPID.reset();
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.FEEDING && feedingElapsedSeconds >= shootDurationSeconds;
    }

    private double interpolateRPM(double distanceMeters) {
        double clamped = Math.max(CLOSE_DISTANCE_METERS, Math.min(FAR_DISTANCE_METERS, distanceMeters));
        double t = (clamped - CLOSE_DISTANCE_METERS) / (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);
        return MIN_RPM + t * (MAX_RPM - MIN_RPM);
    }

    private double interpolateHoodAngle(double distanceMeters) {
        double clamped = Math.max(CLOSE_DISTANCE_METERS, Math.min(FAR_DISTANCE_METERS, distanceMeters));
        double t = (clamped - CLOSE_DISTANCE_METERS) / (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);
        return ShooterSubsystem.CLOSE_SHOT_HOOD
             + t * (ShooterSubsystem.FAR_SHOT_HOOD - ShooterSubsystem.CLOSE_SHOT_HOOD);
    }

    private boolean isHubTag(int tagId) {
        return HUB_TAG_IDS.contains(tagId);
    }
}