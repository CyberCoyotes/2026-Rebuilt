package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

/**
 * AlignToHubCommand - Rotates the robot to face the blue hub AprilTags.
 *
 * BEHAVIOR:
 * - Reads horizontal angle error (tx) from the Limelight via VisionSubsystem
 * - Runs a PID loop to drive tx toward 0 (robot facing the tag)
 * - Applies a minimum output floor so the robot commits to rotating even at large errors
 * - Driver retains full translational control via left stick
 * - Only corrects rotation — never touches X/Y velocity
 * - If no valid tag is seen, rotation correction is zeroed (robot holds heading)
 * - Releases cleanly on button release, returning full control to the driver
 *
 * TUNING:
 * - kP: Main gain — increase if still slow to snap, decrease if oscillating near target
 * - MIN_ROTATION_OUTPUT: Floor that ensures rotation always commits above deadband.
 *   Increase if the robot stalls far from target. Decrease if it overshoots badly.
 * - MAX_ROTATION_RATE: Caps top speed — reduce if alignment is too aggressive
 * - ALIGN_TOLERANCE_DEGREES: Deadband — no correction applied within this range
 *
 * VALID TAG IDS: 18-27 (blue hub), excluding 17, 22, 23, 28
 *
 * @author @Isaak3
 */
public class AlignToHubCommand extends Command {

    // ===== Constants =====

    /** Valid AprilTag ID range for the blue hub */
    private static final int MIN_HUB_TAG_ID = 18;
    private static final int MAX_HUB_TAG_ID = 27;

    /** PID gains for rotation alignment */
    private static final double kP = 0.03; // TODO: Tune
    private static final double kI = 0.0;
    private static final double kD = 0.001;

    /**
     * Minimum rotation output (rad/s) applied whenever error exceeds the deadband.
     * Ensures the robot always commits to rotating even when PID output is weak at large errors.
     * Sign is matched to the direction of error automatically via Math.copySign.
     */
    private static final double MIN_ROTATION_OUTPUT = 0.8; // TODO: Tune

    /** Maximum rotation rate (rad/s) */
    private static final double MAX_ROTATION_RATE = 3.0; // TODO: Tune

    /** Deadband — within this many degrees of tx, no correction applied */
    private static final double ALIGN_TOLERANCE_DEGREES = 2.0;

    // ===== Hardware =====
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;

    // ===== Control =====
    private final PIDController rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    public AlignToHubCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.drivetrain = drivetrain;
        this.vision = vision;
        this.translationX = translationX;
        this.translationY = translationY;

        rotationPID = new PIDController(kP, kI, kD);
        rotationPID.setSetpoint(0.0);
        rotationPID.setTolerance(ALIGN_TOLERANCE_DEGREES);
        rotationPID.enableContinuousInput(-180.0, 180.0);

        driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
    }

    @Override
    public void execute() {
        double rotationOutput = 0.0;

        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double txDegrees = vision.getHorizontalAngleDegrees();

            if (Math.abs(txDegrees) > ALIGN_TOLERANCE_DEGREES) {
                double pidOutput = rotationPID.calculate(txDegrees);

                // Apply minimum output floor — robot always commits to rotating
                // when above the deadband, even if PID math is weak at large errors
                if (Math.abs(pidOutput) < MIN_ROTATION_OUTPUT) {
                    pidOutput = Math.copySign(MIN_ROTATION_OUTPUT, pidOutput);
                }

                rotationOutput = Math.max(-MAX_ROTATION_RATE,
                                 Math.min(MAX_ROTATION_RATE, pidOutput));
            }
        }

        final double finalRotation = rotationOutput;
        drivetrain.applyRequest(() ->
            driveRequest
                .withVelocityX(translationX.getAsDouble())
                .withVelocityY(translationY.getAsDouble())
                .withRotationalRate(finalRotation)
        ).execute();
    }

    @Override
    public void end(boolean interrupted) {
        rotationPID.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Returns true if the tag ID is a valid hub target.
     * Must be within the hub range (18-27) and not in the exclusion list.
     *
     * Excluded tags: 17, 22, 23, 28
     */
    private boolean isHubTag(int tagId) {
        if (tagId == 17 || tagId == 22 || tagId == 23 || tagId == 28) {
            return false;
        }
        return tagId >= MIN_HUB_TAG_ID && tagId <= MAX_HUB_TAG_ID;
    }
}