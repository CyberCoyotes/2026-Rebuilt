package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Set;

public class AutoAlignAndShootCommand extends Command {

    private enum Phase {
        ALIGNING,
        FEEDING
    }

    private static final Set<Integer> HUB_TAG_IDS =
            Set.of(18,19,20,21,24,25,26,27);

    private static final double CLOSE_DISTANCE_METERS = 0.85;
    private static final double FAR_DISTANCE_METERS = 3.5;

    private static final double MIN_RPM = 2875;
    private static final double MAX_RPM = 3500;

    private static final double TX_FILTER_ALPHA = 0.2;

    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.001;

    private static final double MIN_ROTATION_OUTPUT = .25;
    private static final double MAX_ROTATION_RATE = 4.0;

    private static final double ALIGN_TOLERANCE_DEGREES = 1.0;
    private static final double MAX_ALIGN_WAIT_SECONDS = 3.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;

    private final double shootDurationSeconds;

    private final SwerveRequest.FieldCentric alignRequest;
    private final SwerveRequest.SwerveDriveBrake brakeRequest;

    private final PIDController rotationPID;

    private double smoothedTx = 0.0;
    private double lastDistanceMeters = 2.0;

    private Phase currentPhase = Phase.ALIGNING;

    private double phaseStartTime = 0.0;

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
        rotationPID.enableContinuousInput(-180,180);

        alignRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        brakeRequest = new SwerveRequest.SwerveDriveBrake();

        addRequirements(drivetrain, shooter, indexer);
    }

    @Override
    public void initialize() {

        rotationPID.reset();
        smoothedTx = 0;

        currentPhase = Phase.ALIGNING;
        phaseStartTime = Timer.getFPGATimestamp();

        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            lastDistanceMeters = vision.getDistanceToTargetMeters();
            smoothedTx = vision.getHorizontalAngleDegrees();
        }

        shooter.setTargetVelocity(interpolateRPM(lastDistanceMeters));
        shooter.setTargetHoodPose(interpolateHoodAngle(lastDistanceMeters));

        shooter.prepareToShoot();
    }

    @Override
    public void execute() {

        if (currentPhase == Phase.ALIGNING) {

            if (vision.hasTarget() && isHubTag(vision.getTagId())) {

                double rawTx = vision.getHorizontalAngleDegrees();
                smoothedTx = TX_FILTER_ALPHA * rawTx +
                        (1 - TX_FILTER_ALPHA) * smoothedTx;

                lastDistanceMeters = vision.getDistanceToTargetMeters();
            }

            shooter.updateFlywheelVelocity(
                    interpolateRPM(lastDistanceMeters));

            shooter.updateHoodForDistance(
                    interpolateHoodAngle(lastDistanceMeters));

            double rotationOutput = 0;

            if (Math.abs(smoothedTx) > ALIGN_TOLERANCE_DEGREES) {

                double pidOutput = rotationPID.calculate(smoothedTx);

                if (Math.abs(pidOutput) < MIN_ROTATION_OUTPUT) {
                    pidOutput = Math.copySign(MIN_ROTATION_OUTPUT, pidOutput);
                }

                rotationOutput =
                        Math.max(-MAX_ROTATION_RATE,
                        Math.min(MAX_ROTATION_RATE, pidOutput));
            }

            drivetrain.setControl(
                    alignRequest
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(rotationOutput)
            );

            indexer.indexerStop();
            indexer.conveyorStop();

            boolean timeout =
                    Timer.getFPGATimestamp() - phaseStartTime
                            > MAX_ALIGN_WAIT_SECONDS;

            if (shooter.isReady() || timeout) {

                currentPhase = Phase.FEEDING;
                phaseStartTime = Timer.getFPGATimestamp();
            }
        }

        else {

            drivetrain.setControl(brakeRequest);

            indexer.indexerForward();
            indexer.conveyorForward();
        }
    }

    @Override
    public void end(boolean interrupted) {

        indexer.indexerStop();
        indexer.conveyorStop();

        shooter.returnToIdle();

        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {

        if (currentPhase != Phase.FEEDING) return false;

        return Timer.getFPGATimestamp() - phaseStartTime
                >= shootDurationSeconds;
    }

    private double interpolateRPM(double distance) {

        double clamped =
                Math.max(CLOSE_DISTANCE_METERS,
                Math.min(FAR_DISTANCE_METERS, distance));

        double t = (clamped - CLOSE_DISTANCE_METERS) /
                (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);

        return MIN_RPM + t * (MAX_RPM - MIN_RPM);
    }

    private double interpolateHoodAngle(double distance) {

        double clamped =
                Math.max(CLOSE_DISTANCE_METERS,
                Math.min(FAR_DISTANCE_METERS, distance));

        double t = (clamped - CLOSE_DISTANCE_METERS) /
                (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);

        return ShooterSubsystem.CLOSE_SHOT_HOOD +
                t * (ShooterSubsystem.FAR_SHOT_HOOD -
                     ShooterSubsystem.CLOSE_SHOT_HOOD);
    }

    private boolean isHubTag(int id) {
        return HUB_TAG_IDS.contains(id);
    }
}