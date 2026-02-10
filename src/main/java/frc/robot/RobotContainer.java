// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indexer.IndexerCommands;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final GameDataTelemetry gameDataTelemetry = new GameDataTelemetry();

    // ===== Controllers =====
    // Port 0: Driver controller (drivetrain movement)
    // Port 1: Operator controller (mechanisms - shooter, intake, indexer, climber)
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // ===== Subsystems =====
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final LedSubsystem ledSubsystem;
    private final ClimberSubsystem climber;

    // ===== Intake Commands (instance-based, not static) =====
    private final IntakeCommands intakeCommands = new IntakeCommands();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        if (RobotBase.isReal()) {
            intake = new IntakeSubsystem(new IntakeIOHardware());
            indexer = new IndexerSubsystem(new IndexerIOHardware());
            shooter = new ShooterSubsystem(new ShooterIOHardware());
            vision = new VisionSubsystem(new VisionIOLimelight(Constants.Vision.LIMELIGHT3_NAME));
        } else {
            intake = new IntakeSubsystem(new IntakeIOSim());
            indexer = new IndexerSubsystem(new IndexerIOSim());
            shooter = new ShooterSubsystem(new ShooterIOSim());
            vision = new VisionSubsystem(new VisionIOSim());
        }

        ledSubsystem = new LedSubsystem(shooter);
        climber = new ClimberSubsystem();

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        // AutoChooser automatically publishes to NetworkTables at "AutoChooser"
        // Elastic dashboard can read this directly without SmartDashboard

        configureBindings();
    }

    private void configureBindings() {
        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Drivetrain
        // =====================================================================

        // Default: Field-centric drive with left stick (translate) and right stick (rotate)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // A: Brake (lock wheels in X pattern)
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // B: Point wheels at joystick direction (for testing)
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // POV Up/Down: Slow manual drive (for alignment/testing)
        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Left Bumper: Reset field-centric heading
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // SysId routines (Back/Start + X/Y)
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);

        // =====================================================================
        // OPERATOR CONTROLLER (Port 1) - Mechanisms
        // =====================================================================

        // ----- Shooter -----
        // Right Trigger: Spin up shooter (pre-rev while held)
        operator.rightTrigger(0.5).whileTrue(ShooterCommands.spinUp(shooter));

        // Y: Close shot (prepare and wait for ready)
        operator.y().onTrue(ShooterCommands.closeShot(shooter));

        // X: Far shot (prepare and wait for ready)
        operator.x().onTrue(ShooterCommands.farShot(shooter));

        // B: Shooter idle (stop all)
        operator.b().onTrue(ShooterCommands.idle(shooter));

        // POV Up: Eject from shooter (clear jams, 1 second reverse)
        operator.povUp().onTrue(ShooterCommands.eject(shooter, 1.0));

        // ----- Indexer -----
        // Right Bumper: Feed game piece to shooter (while held)
        operator.rightBumper().whileTrue(IndexerCommands.feed(indexer));

        // POV Down: Eject from indexer (clear jams, 1 second reverse)
        operator.povDown().onTrue(IndexerCommands.eject(indexer, 1.0));

        // ----- Intake -----
        // Left Trigger: Run intake (while held)
        operator.leftTrigger(0.5).whileTrue(intakeCommands.enterIntakeMode(intake));

        // Left Bumper: Stop intake jam (quick reverse)
        operator.leftBumper().onTrue(intakeCommands.stopJam(intake));

        // ----- Full Sequences -----
        // A: Full shoot sequence - close shot with indexer feed, then idle
        operator.a().onTrue(
            ShooterCommands.shootSequence(shooter, indexer,
                ShooterSubsystem.CLOSE_SHOT_RPM, ShooterSubsystem.CLOSE_SHOT_ANGLE)
        );

        // ----- Climber (POV Left/Right) -----
        // POV Right: Extend climber arm (while held)
        operator.povRight().whileTrue(climber.extendArm());

        // POV Left: Retract climber arm (while held)
        operator.povLeft().whileTrue(climber.retractArm());

        // Start: Stop climber
        operator.start().onTrue(climber.stopClimber());
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }

    /**
     * Updates game data telemetry. Call this in robotPeriodic().
     * Polls for FMS game-specific message and publishes to NetworkTables.
     */
    public void updateGameData() {
        gameDataTelemetry.update();
    }

    /**
     * Returns the game data telemetry instance for programmatic access.
     */
    public GameDataTelemetry getGameDataTelemetry() {
        return gameDataTelemetry;
    }
}
