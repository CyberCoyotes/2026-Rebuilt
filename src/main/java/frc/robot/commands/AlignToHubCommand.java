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
 * - Driver retains full translational control via left stick
 * - Only corrects rotation — never touches X/Y velocity
 * - If no valid tag is seen, rotation correction is zeroed (robot holds heading)
 * - Releases cleanly on button release, returning full control to the driver
 *
 * TUNING:
 * - Increase kP if the robot is slow to snap to the target
 * - Decrease kP if the robot oscillates around the target
 * - Add kD to dampen oscillation once kP is set
 * - ALIGN_TOLERANCE_DEGREES controls the deadband — within this range, no correction is applied
 *
 * USAGE:
 *   driver.povLeft().whileTrue(new AlignToHubCommand(drivetrain, vision,
 *       () -> -driver.getLeftY(),
 *       () -> -driver.getLeftX()));
 *
 * VALID TAG IDS: 23-28 (blue hub)
 *
 * @author @Isaak3
 */
public class AlignToHubCommand extends Command {

    // ===== Constants =====

    /** Valid AprilTag IDs for the blue hub */
    private static final int MIN_HUB_TAG_ID = 18;
    private static final int MAX_HUB_TAG_ID = 27;

    /**
     * PID gains for rotation alignment.
     * kP: degrees of tx -> radians/s of rotation output
     * Start low (0.03) and increase until snappy without oscillation.
     */
    private static final double kP = 0.01;  // TODO: Tune — increase if too slow, decrease if oscillating
    private static final double kI = 0.0;   // Leave at 0 until kP is tuned
    private static final double kD = 0.001; // TODO: Tune — helps dampen oscillation

    /**
     * Deadband — within this many degrees of tx, no rotation correction is applied.
     * Prevents jitter when already well-aligned.
     */
    private static final double ALIGN_TOLERANCE_DEGREES = 2.0; // TODO: Tune for shot consistency

    /** Maximum rotation rate output (rad/s) — caps how fast the robot can spin to align */
    private static final double MAX_ROTATION_RATE = 1.5; // TODO: Tune — reduce if alignment feels jerky

    // ===== Hardware =====
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final DoubleSupplier translationX; // Forward/back from left stick Y
    private final DoubleSupplier translationY; // Strafe from left stick X

    // ===== Control =====
    private final PIDController rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    /**
     * Creates a new AlignToHubCommand.
     *
     * @param drivetrain   The swerve drivetrain subsystem
     * @param vision       The vision subsystem
     * @param translationX Supplier for forward/back velocity (left stick Y, negated)
     * @param translationY Supplier for strafe velocity (left stick X, negated)
     */
    public AlignToHubCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.drivetrain = drivetrain;
        this.vision = vision;
        this.translationX = translationX;
        this.translationY = translationY;

        // PID controller — setpoint is 0 degrees (robot facing the tag)
        rotationPID = new PIDController(kP, kI, kD);
        rotationPID.setSetpoint(0.0);
        rotationPID.setTolerance(ALIGN_TOLERANCE_DEGREES);
        rotationPID.enableContinuousInput(-180.0, 180.0);

        // Reuse field-centric drive request — same as default drive but we override rotation
        driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(0.0) // No deadband — PID handles output smoothing
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

        // Only correct rotation if we have a valid hub tag in view
        if (vision.hasTarget() && isHubTag(vision.getTagId())) {
            double txDegrees = vision.getHorizontalAngleDegrees();

            // Apply deadband — don't correct if already within tolerance
            if (Math.abs(txDegrees) > ALIGN_TOLERANCE_DEGREES) {
                rotationOutput = rotationPID.calculate(txDegrees);

                // Clamp output to max rotation rate
                rotationOutput = Math.max(-MAX_ROTATION_RATE, Math.min(MAX_ROTATION_RATE, rotationOutput));
            }
        }
        // If no valid tag, rotationOutput stays 0 — robot holds its current heading naturally

        // Apply field-centric drive with driver translation + PID rotation
    
        final double finalRotationOutput = rotationOutput;
drivetrain.applyRequest(() ->
    driveRequest
        .withVelocityX(translationX.getAsDouble())
        .withVelocityY(translationY.getAsDouble())
        .withRotationalRate(finalRotationOutput)
).execute();
}

    @Override
    public void end(boolean interrupted) {
        // Stop rotation on release — driver regains full control immediately
        rotationPID.reset();
    }

    @Override
    public boolean isFinished() {
        return false; // Run until button is released (whileTrue handles cancellation)
    }

    /**
     * Returns true if the given tag ID belongs to the blue hub (tags 23-28).
     */
    private boolean isHubTag(int tagId) {
        return tagId >= MIN_HUB_TAG_ID && tagId <= MAX_HUB_TAG_ID;
    }
}