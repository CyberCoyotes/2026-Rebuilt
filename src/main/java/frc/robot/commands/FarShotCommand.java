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
import java.util.function.DoubleSupplier;

public class FarShotCommand extends Command {

    // ===== Hub Tags =====
    private static final Set<Integer> HUB_TAG_IDS = Set.of(25, 27, 24);

    // ===== Distance Range =====
    private static final double CLOSE_DISTANCE_METERS = 0.85;
    private static final double FAR_DISTANCE_METERS = 3.25;

    // ===== RPM Range =====
    private static final double MIN_RPM = 2875;
    private static final double MAX_RPM = 3500;

    // ===== Filter =====
    private static final double TX_FILTER_ALPHA = 0.2;

    // ===== Rotation PID =====
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.001;

    private static final double MIN_ROTATION_OUTPUT = 1.75;
    private static final double MAX_ROTATION_RATE = 4.0;
    private static final double ALIGN_TOLERANCE_DEGREES = 4.0;

    // ===== Tag Lock =====
    private int lockedTagId = -1;
    private int lostTagFrames = 0;
    private static final int LOST_TAG_THRESHOLD = 10;

    // ===== Hardware =====
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;

    private final PIDController rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    private boolean isFeeding = false;
    private double smoothedTx = 0.0;

    public FarShotCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            IndexerSubsystem indexer,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.vision = vision;
        this.indexer = indexer;
        this.translationX = translationX;
        this.translationY = translationY;

        rotationPID = new PIDController(kP, kI, kD);
        rotationPID.setSetpoint(0.0);
        rotationPID.setTolerance(ALIGN_TOLERANCE_DEGREES);
        rotationPID.enableContinuousInput(-180.0, 180.0);

        driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, shooter, vision, indexer);
    }

    @Override
    public void initialize() {

        rotationPID.reset();
        isFeeding = false;

        lockedTagId = -1;
        lostTagFrames = 0;

        if (vision.hasTarget() && isHubTag(vision.getTagId())) {

            lockedTagId = vision.getTagId();
            smoothedTx = vision.getHorizontalAngleDegrees();

            double distance = vision.getDistanceToTargetMeters();

            shooter.setTargetVelocity(interpolateRPM(distance));
            shooter.setTargetHoodPose(interpolateHoodAngle(distance));

        } else {

            smoothedTx = 0.0;
            shooter.setTargetVelocity(MIN_RPM);
            shooter.setTargetHoodPose(ShooterSubsystem.CLOSE_SHOT_HOOD);

        }

        shooter.prepareToShoot();
    }

    @Override
    public void execute() {

        // ============================================================
        // TAG LOCK MANAGEMENT
        // ============================================================

        boolean hasLockedTag = vision.hasTarget() && vision.getTagId() == lockedTagId;

        if (!hasLockedTag) {

            lostTagFrames++;

            if (lostTagFrames > LOST_TAG_THRESHOLD) {

                if (vision.hasTarget() && isHubTag(vision.getTagId())) {

                    lockedTagId = vision.getTagId();
                    lostTagFrames = 0;
                }
            }

        } else {
            lostTagFrames = 0;
        }

        // ============================================================
        // UPDATE TARGETING
        // ============================================================

        if (vision.hasTarget() && vision.getTagId() == lockedTagId) {

            double rawTx = vision.getHorizontalAngleDegrees();
            smoothedTx = (TX_FILTER_ALPHA * rawTx) + ((1.0 - TX_FILTER_ALPHA) * smoothedTx);

            double distance = vision.getDistanceToTargetMeters();

            shooter.updateHoodForDistance(interpolateHoodAngle(distance));
            shooter.updateFlywheelVelocity(interpolateRPM(distance));
        }

        // ============================================================
        // ROTATION CONTROL
        // ============================================================

        double rotationOutput = 0.0;

        if (vision.hasTarget() && vision.getTagId() == lockedTagId) {

            if (Math.abs(smoothedTx) > ALIGN_TOLERANCE_DEGREES) {

                double pidOutput = rotationPID.calculate(smoothedTx);

                if (Math.abs(pidOutput) < MIN_ROTATION_OUTPUT) {
                    pidOutput = Math.copySign(MIN_ROTATION_OUTPUT, pidOutput);
                }

                rotationOutput = Math.max(-MAX_ROTATION_RATE,
                        Math.min(MAX_ROTATION_RATE, pidOutput));

                // damping near target to remove shake
                if (Math.abs(smoothedTx) < 2.5) {
                    rotationOutput *= 0.5;
                }
            }
        }

        // ============================================================
        // DRIVE
        // ============================================================

        final double finalRotation = rotationOutput;

        drivetrain.applyRequest(() ->
                driveRequest
                        .withVelocityX(translationX.getAsDouble())
                        .withVelocityY(translationY.getAsDouble())
                        .withRotationalRate(finalRotation)
        ).execute();

        // ============================================================
        // FEEDING
        // ============================================================

        if (shooter.isReady()) {

            isFeeding = true;
            indexer.indexerForward();
            indexer.conveyorForward();

        } else if (!isFeeding) {

            indexer.indexerStop();
            indexer.conveyorStop();
        }
    }

    @Override
    public void end(boolean interrupted) {

        indexer.indexerStop();
        indexer.conveyorStop();
        shooter.returnToIdle();
        rotationPID.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double interpolateRPM(double distanceMeters) {

        double clamped = Math.max(CLOSE_DISTANCE_METERS,
                Math.min(FAR_DISTANCE_METERS, distanceMeters));

        double t = (clamped - CLOSE_DISTANCE_METERS)
                / (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);

        return MIN_RPM + t * (MAX_RPM - MIN_RPM);
    }

    private double interpolateHoodAngle(double distanceMeters) {

        double clamped = Math.max(CLOSE_DISTANCE_METERS,
                Math.min(FAR_DISTANCE_METERS, distanceMeters));

        double t = (clamped - CLOSE_DISTANCE_METERS)
                / (FAR_DISTANCE_METERS - CLOSE_DISTANCE_METERS);

        return ShooterSubsystem.CLOSE_SHOT_HOOD
                + t * (ShooterSubsystem.FAR_SHOT_HOOD - ShooterSubsystem.CLOSE_SHOT_HOOD);
    }

    private boolean isHubTag(int tagId) {
        return HUB_TAG_IDS.contains(tagId);
    }
}