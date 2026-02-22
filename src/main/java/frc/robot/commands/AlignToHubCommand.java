package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * AlignToHubCommand
 *
 * Wiring (in RobotContainer):
 *   X button  → InstantCommand(shooter::setHubShotPreset)
 *                 Arms the flywheel and hood at hub defaults while the robot is still moving.
 *   RT        → toggleOnTrue(new AlignToHubCommand(...))
 *                 Only fires when isHubShotArmed() is true (guarded by Trigger in RobotContainer).
 *                 First press starts aligning and shoots; second press cancels cleanly.
 *
 * ROTATION STRATEGY:
 *   Reads tx and distance from VisionSubsystem, which works in both sim
 *   (via VisionIOSim) and on real hardware (via VisionIOLimelight).
 *   No direct LimelightHelpers calls.
 *
 * PHASES:
 *   ALIGNING — rotates toward hub center, updates flywheel RPM and hood angle from
 *              live distance. Transitions to FEEDING when aligned + shooter ready,
 *              or after MAX_ALIGN_SECONDS. Will NOT fire blind on target loss.
 *   FEEDING  — locks drivetrain, runs indexer + conveyor for shootDurationSeconds.
 */
public class AlignToHubCommand extends Command {

    // -------------------------------------------------------------------------
    // Distance & RPM interpolation
    // -------------------------------------------------------------------------

    private static final double CLOSE_DISTANCE_M = 0.85;
    private static final double FAR_DISTANCE_M   = 3.50;

    private static final double MIN_RPM = 2875;
    private static final double MAX_RPM = 3500;

    // -------------------------------------------------------------------------
    // Rotation PID
    // -------------------------------------------------------------------------

    private static final double ROT_kP              = 0.03;
    private static final double ROT_kI              = 0.00;
    private static final double ROT_kD              = 0.002;
    private static final double ROT_MIN_OUTPUT      = 2.0;
    private static final double ROT_MAX_OUTPUT      = 3.5;
    private static final double ALIGN_TOLERANCE_DEG = 0.5;

    // -------------------------------------------------------------------------
    // Timing
    // -------------------------------------------------------------------------

    private static final double MAX_ALIGN_SECONDS = 3.0;
    private static final double LOOP_PERIOD_S     = 0.02;

    // -------------------------------------------------------------------------
    // Tx smoothing
    // -------------------------------------------------------------------------

    private static final int TX_FILTER_TAPS = 5;

    // -------------------------------------------------------------------------
    // Phase
    // -------------------------------------------------------------------------

    private enum Phase { ALIGNING, FEEDING }

    // -------------------------------------------------------------------------
    // Dependencies
    // -------------------------------------------------------------------------

    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem        shooter;
    private final IndexerSubsystem        indexer;
    private final VisionSubsystem         vision;
    private final double                  shootDurationSeconds;

    // -------------------------------------------------------------------------
    // Swerve requests
    // -------------------------------------------------------------------------

    private final SwerveRequest.FieldCentric     rotateRequest;
    private final SwerveRequest.SwerveDriveBrake brakeRequest;

    // -------------------------------------------------------------------------
    // Control objects
    // -------------------------------------------------------------------------

    private final PIDController rotationPID;
    private final LinearFilter  txFilter;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------

    private Phase  phase;
    private double alignTimer;
    private double feedTimer;
    private double smoothedTx;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * @param drivetrain           CTRE swerve drivetrain
     * @param shooter              Shooter subsystem (flywheel + hood)
     * @param indexer              Indexer subsystem (indexer + conveyor)
     * @param vision               VisionSubsystem (works in sim and on hardware)
     * @param shootDurationSeconds Seconds to run indexer/conveyor during FEEDING
     */
    public AlignToHubCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            VisionSubsystem vision,
            double shootDurationSeconds) {

        this.drivetrain           = drivetrain;
        this.shooter              = shooter;
        this.indexer              = indexer;
        this.vision               = vision;
        this.shootDurationSeconds = shootDurationSeconds;

        rotationPID = new PIDController(ROT_kP, ROT_kI, ROT_kD);
        rotationPID.setSetpoint(0.0);
        rotationPID.setTolerance(ALIGN_TOLERANCE_DEG);
        rotationPID.enableContinuousInput(-180.0, 180.0);

        txFilter = LinearFilter.movingAverage(TX_FILTER_TAPS);

        rotateRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        brakeRequest = new SwerveRequest.SwerveDriveBrake();

        addRequirements(drivetrain, shooter, indexer);
    }

    // -------------------------------------------------------------------------
    // Command lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void initialize() {
        rotationPID.reset();
        txFilter.reset();

        phase      = Phase.ALIGNING;
        alignTimer = 0.0;
        feedTimer  = 0.0;
        smoothedTx = 0.0;

        shooter.setHubShotPreset();
        shooter.refreshReady();

        // Seed filter with first real reading so we don't snap toward zero
        if (vision.hasTarget()) {
            smoothedTx = txFilter.calculate(vision.getRawTxDegrees());
        }
    }

    @Override
    public void execute() {
        switch (phase) {
            case ALIGNING: runAligning(); break;
            case FEEDING:  runFeeding();  break;
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
        return phase == Phase.FEEDING && feedTimer >= shootDurationSeconds;
    }

    // -------------------------------------------------------------------------
    // Phase: ALIGNING
    // -------------------------------------------------------------------------

    private void runAligning() {
        alignTimer += LOOP_PERIOD_S;

        boolean hasTarget = vision.hasTarget();

        if (hasTarget) {
            smoothedTx = txFilter.calculate(vision.getRawTxDegrees());

            // Update shooter based on distance from VisionSubsystem
            double dist = vision.getDistanceToHub();
            if (dist > 0) {
                shooter.updateFlywheelVelocity(lerpRPM(dist));
                shooter.updateHoodForDistance(lerpHood(dist));
            }
        }
        // If no target, hold last smoothedTx and don't update shooter

        // Compute rotation output
        double rotOutput = 0.0;
        if (hasTarget && Math.abs(smoothedTx) > ALIGN_TOLERANCE_DEG) {
            double raw = -rotationPID.calculate(smoothedTx);
            raw = Math.abs(raw) < ROT_MIN_OUTPUT
                    ? Math.copySign(ROT_MIN_OUTPUT, raw)
                    : raw;
            rotOutput = Math.max(-ROT_MAX_OUTPUT, Math.min(ROT_MAX_OUTPUT, raw));
        }

        drivetrain.setControl(
            rotateRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(rotOutput)
        );

        indexer.indexerStop();
        indexer.conveyorStop();

        boolean aligned  = hasTarget && Math.abs(smoothedTx) <= ALIGN_TOLERANCE_DEG;
        boolean timedOut = alignTimer >= MAX_ALIGN_SECONDS;

        if ((aligned && shooter.isReady()) || timedOut) {
            phase = Phase.FEEDING;
        }
    }

    // -------------------------------------------------------------------------
    // Phase: FEEDING
    // -------------------------------------------------------------------------

    private void runFeeding() {
        drivetrain.setControl(brakeRequest);
        indexer.indexerForward();
        indexer.conveyorForward();
        feedTimer += LOOP_PERIOD_S;
    }

    // -------------------------------------------------------------------------
    // Interpolation
    // -------------------------------------------------------------------------

    private double lerpRPM(double distM) {
        return MIN_RPM + t(distM) * (MAX_RPM - MIN_RPM);
    }

    private double lerpHood(double distM) {
        return ShooterSubsystem.CLOSE_SHOT_HOOD
             + t(distM) * (ShooterSubsystem.FAR_SHOT_HOOD - ShooterSubsystem.CLOSE_SHOT_HOOD);
    }

    private double t(double distM) {
        double clamped = Math.max(CLOSE_DISTANCE_M, Math.min(FAR_DISTANCE_M, distM));
        return (clamped - CLOSE_DISTANCE_M) / (FAR_DISTANCE_M - CLOSE_DISTANCE_M);
    }
}