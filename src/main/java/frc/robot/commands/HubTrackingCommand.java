package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class HubTrackingCommand extends Command {

    private static final double ROT_kP             = 0.15;  // TODO: Tune
    private static final double ROT_kI             = 0.00;
    private static final double ROT_kD             = 0.00;
    private static final double ROT_MAX_OUTPUT     = 3.0;   // rad/s
    private static final double TOLERANCE_DEG      = 1.0;
    private static final double MAX_TRACKING_SPEED = 1.5;   // m/s

    private final CommandSwerveDrivetrain           drivetrain;
    private final ShooterSubsystem                  shooter;
    private final IndexerSubsystem                  indexer;
    private final VisionSubsystem                   vision;
    private final java.util.function.DoubleSupplier vx;
    private final java.util.function.DoubleSupplier vy;

    private final PIDController              pid;
    private final SwerveRequest.FieldCentric request;

    public HubTrackingCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            VisionSubsystem vision,
            java.util.function.DoubleSupplier vx,
            java.util.function.DoubleSupplier vy) {

        this.drivetrain = drivetrain;
        this.shooter    = shooter;
        this.indexer    = indexer;
        this.vision     = vision;
        this.vx         = vx;
        this.vy         = vy;

        pid = new PIDController(ROT_kP, ROT_kI, ROT_kD);
        pid.setSetpoint(0.0);
        pid.setTolerance(TOLERANCE_DEG);
        pid.enableContinuousInput(-180.0, 180.0);

        request = new SwerveRequest.FieldCentric()
            .withDeadband(0.0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain, shooter, indexer);
    }

    @Override
    public void initialize() {
        pid.reset();
        shooter.setHubShotPreset();
        shooter.prepareToShoot();
    }

    @Override
    public void execute() {
        // Update shooter from distance
        double dist = vision.getDistanceToHub();
        if (dist > 0) {
            shooter.updateFromDistance(dist);
        }

        // Rotate toward hub
        double angle = vision.getAngleToHub();
        double rot   = pid.calculate(angle);
        rot = Math.max(-ROT_MAX_OUTPUT, Math.min(ROT_MAX_OUTPUT, rot));

        drivetrain.setControl(
            request
                .withVelocityX(cap(vx.getAsDouble()))
                .withVelocityY(cap(vy.getAsDouble()))
                .withRotationalRate(rot)
        );

        // Feed once shooter is ready — stop if it drops out of tolerance
        if (shooter.isReady()) {
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
        shooter.returnToIdle();
        pid.reset();
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double cap(double v) {
        return Math.max(-MAX_TRACKING_SPEED, Math.min(MAX_TRACKING_SPEED, v));
    }
}