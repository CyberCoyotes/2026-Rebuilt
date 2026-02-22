package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.Set;

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
 * ROTATION STRATEGY — two-path approach:
 *   PRIMARY   — JSON multi-tag weighted average. Reads all visible hub AprilTags from
 *               getLatestResults(), weights each tag's tx and distance by its area.
 *   FALLBACK  — Raw NT getTX(). Used when JSON parsing returns no valid hub tags.
 *               Distance interpolation is skipped in fallback mode.
 *
 * PHASES:
 *   ALIGNING — rotates toward hub center, updates flywheel RPM and hood angle from
 *              live distance estimate (primary path only). Transitions to FEEDING when
 *              aligned + shooter ready, or after MAX_ALIGN_SECONDS. Will NOT fire blind
 *              on target loss — holds position and waits for timeout.
 *   FEEDING  — locks drivetrain, runs indexer + conveyor for shootDurationSeconds.
 */
public class AlignToHubCommand extends Command {

    // -------------------------------------------------------------------------
    // Limelight & field constants
    // -------------------------------------------------------------------------

    private static final String LIMELIGHT_NAME = "limelight-four";

    /** FRC 2025 Reefscape hub-facing tag IDs — adjust if your layout differs. */
    private static final Set<Integer> HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    // -------------------------------------------------------------------------
    // Distance & RPM interpolation
    // -------------------------------------------------------------------------

    private static final double CLOSE_DISTANCE_M = 0.85;
    private static final double FAR_DISTANCE_M   = 3.50;

    private static final double MIN_RPM = 2875;
    private static final double MAX_RPM = 3500;

    // -------------------------------------------------------------------------
    // Camera mounting geometry
    // -------------------------------------------------------------------------

    private static final double LL4_HEIGHT_M     = 0.533;  // lens height from floor (21" = 0.533m)
    private static final double LL4_ANGLE_DEG    = 10.0;   // upward tilt from horizontal (deg)
    private static final double HUB_TAG_HEIGHT_M = 1.143;  // hub tag center height from floor (45" = 1.143m)

    // -------------------------------------------------------------------------
    // Rotation PID
    //
    // kP tuning guide:
    //   Too high → blows past target and oscillates (what we saw)
    //   Too low  → stops short of target
    //   Start at 0.03 and increase by 0.01 until it snaps cleanly without overshooting.
    //
    // ROT_MIN_OUTPUT tuning guide:
    //   Too high → always overshoots even when close
    //   Too low  → stalls before reaching target
    //   2.0 rad/s is a safe starting point for most swerve drives.
    // -------------------------------------------------------------------------

    private static final double ROT_kP              = 0.03;  // lowered — was overshooting at 0.08
    private static final double ROT_kI              = 0.00;
    private static final double ROT_kD              = 0.002; // slight bump to dampen overshoot
    private static final double ROT_MIN_OUTPUT      = 2.0;   // lowered — 3.0 was too aggressive
    private static final double ROT_MAX_OUTPUT      = 3.5;   // lowered slightly for smoother approach
    private static final double ALIGN_TOLERANCE_DEG = 0.5;   // tightened from 1.0 — stops closer to center

    // -------------------------------------------------------------------------
    // Timing
    // -------------------------------------------------------------------------

    private static final double MAX_ALIGN_SECONDS = 3.0;
    private static final double LOOP_PERIOD_S     = 0.02;  // 50 Hz robot loop

    // -------------------------------------------------------------------------
    // Tx smoothing — 5-tap moving average ≈ 100 ms window at 50 Hz
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
     * @param shootDurationSeconds Seconds to run indexer/conveyor during FEEDING
     */
    public AlignToHubCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            double shootDurationSeconds) {

        this.drivetrain           = drivetrain;
        this.shooter              = shooter;
        this.indexer              = indexer;
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

        // Seed filter with first real reading so we don't rotate toward zero.
        HubSnapshot seed = readHub();
        if (seed.valid) {
            smoothedTx = txFilter.calculate(seed.weightedTx);
        } else if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            smoothedTx = txFilter.calculate(LimelightHelpers.getTX(LIMELIGHT_NAME));
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

        HubSnapshot snap = readHub();
        boolean hasTarget;

        if (snap.valid) {
            // PRIMARY PATH — multi-tag JSON weighted average
            smoothedTx = txFilter.calculate(snap.weightedTx);
            shooter.updateFlywheelVelocity(lerpRPM(snap.distance));
            shooter.updateHoodForDistance(lerpHood(snap.distance));
            hasTarget = true;
        } else if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            // FALLBACK PATH — raw NT tx, no distance data so skip interpolation
            smoothedTx = txFilter.calculate(LimelightHelpers.getTX(LIMELIGHT_NAME));
            hasTarget = true;
        } else {
            // No target at all — hold last smoothedTx, don't update shooter
            hasTarget = false;
        }

        // Compute rotation output.
        // Negated because a positive tx (target is to the right) requires a negative
        // rotational rate (rotate clockwise / right) in WPILib field-centric convention.
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

        // Only transition to FEEDING if we have a confirmed target and are aligned.
        // Never fire blind on target loss — let the timeout handle it.
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
    // Limelight read — weighted average across all visible hub tags
    // -------------------------------------------------------------------------

    private HubSnapshot readHub() {
        LimelightResults results = LimelightHelpers.getLatestResults(LIMELIGHT_NAME);
        if (results == null) return HubSnapshot.INVALID;

        LimelightTarget_Fiducial[] tags = results.targets_Fiducials;
        if (tags == null || tags.length == 0) return HubSnapshot.INVALID;

        double txSum     = 0.0;
        double distSum   = 0.0;
        double weightSum = 0.0;
        int    count     = 0;

        for (LimelightTarget_Fiducial tag : tags) {
            if (!HUB_TAG_IDS.contains((int) tag.fiducialID)) continue;
            if (tag.ta <= 0.0) continue;

            double dist = distanceFromTy(tag.ty);
            txSum     += tag.tx * tag.ta;
            distSum   += dist   * tag.ta;
            weightSum += tag.ta;
            count++;
        }

        if (count == 0 || weightSum <= 0.0) return HubSnapshot.INVALID;

        return new HubSnapshot(txSum / weightSum, distSum / weightSum);
    }

    // -------------------------------------------------------------------------
    // Distance from vertical angle
    // -------------------------------------------------------------------------

    private double distanceFromTy(double tyDeg) {
        double angleDeg = LL4_ANGLE_DEG + tyDeg;
        if (angleDeg <= 0.0) return FAR_DISTANCE_M;
        return (HUB_TAG_HEIGHT_M - LL4_HEIGHT_M) / Math.tan(Math.toRadians(angleDeg));
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

    /** Normalised position of distM within [CLOSE, FAR], clamped to [0, 1]. */
    private double t(double distM) {
        double clamped = Math.max(CLOSE_DISTANCE_M, Math.min(FAR_DISTANCE_M, distM));
        return (clamped - CLOSE_DISTANCE_M) / (FAR_DISTANCE_M - CLOSE_DISTANCE_M);
    }

    // -------------------------------------------------------------------------
    // HubSnapshot — immutable value from a single Limelight read
    // -------------------------------------------------------------------------

    private static final class HubSnapshot {
        static final HubSnapshot INVALID = new HubSnapshot(0.0, 0.0, false);

        final double  weightedTx;
        final double  distance;
        final boolean valid;

        HubSnapshot(double weightedTx, double distance) {
            this(weightedTx, distance, true);
        }

        HubSnapshot(double weightedTx, double distance, boolean valid) {
            this.weightedTx = weightedTx;
            this.distance   = distance;
            this.valid      = valid;
        }
    }
}