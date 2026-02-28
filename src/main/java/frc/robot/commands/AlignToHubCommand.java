package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

/**
 * AlignToHubCommand - Rotates the robot to face the hub using pose-based targeting.
 *
 * BEHAVIOR:
 * - Uses VisionSubsystem.getAngleToHub() — pose-derived angle error from robot to
 *   Constants.Vision.HUB_CENTER_BLUE — rather than raw Limelight tx.
 * - Runs a PID loop to drive the angle error toward 0 (robot facing the hub).
 * - Applies a minimum output floor so the robot commits to rotating even at large errors.
 * - Driver retains full translational control via left stick.
 * - If no valid pose has been accepted yet, rotation correction is zeroed.
 * - Releases cleanly on button release, returning full control to the driver.
 *
 * TUNING:
 * - kP: Main gain — increase if slow to snap, decrease if oscillating near target
 * - MIN_ROTATION_OUTPUT: Floor that ensures rotation always commits above deadband.
 *   Increase if robot stalls far from target. Decrease if it overshoots badly.
 * - MAX_ROTATION_RATE: Caps top speed — reduce if alignment is too aggressive
 * - ALIGN_TOLERANCE_DEGREES: Deadband — no correction applied within this range
 */
public class AlignToHubCommand extends Command {

    // ===== Tuning Constants =====

    /** PID gains for rotation alignment */
    private static final double kP = 0.05; // TODO: Tune
    private static final double kI = 0.0;
    private static final double kD = 0.001;

    /**
     * Minimum rotation output (rad/s) applied whenever error exceeds the deadband.
     * Sign is matched to the direction of error via Math.copySign.
     */
    private static final double MIN_ROTATION_OUTPUT = 1.25; // TODO: Tune

    /** Maximum rotation rate (rad/s) */
    private static final double MAX_ROTATION_RATE = 4.0; // TODO: Tune

    /** Deadband — within this many degrees of error, no correction applied */
    private static final double ALIGN_TOLERANCE_DEGREES = 1.5;

    // ===== Hardware =====

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem         vision;
    private final DoubleSupplier          translationX;
    private final DoubleSupplier          translationY;

    // ===== Control =====

    private final PIDController              rotationPID;
    private final SwerveRequest.FieldCentric driveRequest;

    public AlignToHubCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier translationX,
            DoubleSupplier translationY) {

        this.drivetrain   = drivetrain;
        this.vision       = vision;
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

        // Only correct rotation if we have a valid accepted pose to work from
        if (vision.hasPose()) {
            double angleErrorDeg = vision.getAngleToHub();

            if (Math.abs(angleErrorDeg) > ALIGN_TOLERANCE_DEGREES) {
                double pidOutput = rotationPID.calculate(angleErrorDeg);

                // Apply minimum output floor — robot always commits to rotating
                // when above the deadband, even if PID math is weak at large errors
                if (Math.abs(pidOutput) < MIN_ROTATION_OUTPUT) {
                    pidOutput = Math.copySign(MIN_ROTATION_OUTPUT, pidOutput);
                }

                rotationOutput = Math.max(-MAX_ROTATION_RATE,
                                 Math.min( MAX_ROTATION_RATE, pidOutput));
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
}