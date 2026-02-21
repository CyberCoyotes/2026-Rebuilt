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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.commands.IndexerCommands;
// import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
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

@SuppressWarnings("unused") // Suppress warnings for unused right now

public class RobotContainer {
    /* TODO Lower max speed and angular rate for testing,
    * then increase to actual desired values once we have tested the robot's responsiveness and handling at lower speeds.
    * This will help prevent runaway situations while tuning.*/
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 0.5 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final GameDataTelemetry gameDataTelemetry = new GameDataTelemetry();

    // ===== Controllers =====
    // Port 0: Driver controller
    private final CommandXboxController driver = new CommandXboxController(0);

    // Port 1: Operator controller
    private final CommandXboxController operator = new CommandXboxController(1);

    // ===== Subsystems =====
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final LedSubsystem ledSubsystem;
    // private final ClimberSubsystem climber;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        intake = new IntakeSubsystem(new IntakeIOHardware());
        indexer = new IndexerSubsystem(new IndexerIOHardware());
        shooter = new ShooterSubsystem(new ShooterIOHardware());
        vision = new VisionSubsystem(new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME));

        ledSubsystem = new LedSubsystem();
        // climber = new ClimberSubsystem();

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);

        configureBindings();
    }

    private void configureBindings() {
        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Drivetrain
        // =====================================================================

        // Default: Field-centric drive with left stick (translate) and right stick (rotate)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // Idle drive motors while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //Start: Reset field-centric heading
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Mechanisms
        // =====================================================================

        // Right Trigger: Shoot — ramp flywheel to target RPM and feed indexer while held
        driver.rightTrigger(0.5).whileTrue(
            ShooterCommands.rampTestShoot(shooter, indexer)
        );

        // Left Trigger: Intake — deploy and run intake rollers while held
        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // Left Bumper: Hood angle up — increases hood pose by HOOD_TEST_INCREMENT rotations.
        // If shooter is in READY state the hood moves immediately; otherwise the new
        // target takes effect on the next prepareToShoot() call.
        driver.leftBumper().onTrue(
            ShooterCommands.increaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT)
        );

        // Right Bumper: Hood angle down — decreases hood pose by HOOD_TEST_INCREMENT rotations.
        driver.rightBumper().onTrue(
            ShooterCommands.decreaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT)
        );

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Commented out (TODO: enable as needed)
        // =====================================================================

        // A: Close shot hood preset
        driver.a().onTrue(Commands.runOnce(() -> {
            shooter.setTargetHoodPose(ShooterSubsystem.CLOSE_SHOT_HOOD);
            shooter.prepareToShoot();
        }));

        // X: Far shot hood preset
        driver.x().onTrue(Commands.runOnce(() -> {
            shooter.setTargetHoodPose(ShooterSubsystem.FAR_SHOT_HOOD);
            shooter.prepareToShoot();
        }));

        // B: Pass shot hood preset
        driver.b().onTrue(Commands.runOnce(() -> {
            shooter.setTargetHoodPose(ShooterSubsystem.PASS_SHOT_HOOD);
            shooter.prepareToShoot();
        }));

        // Y: Indexer forward while held
        // driver.y().whileTrue(Commands.startEnd(indexer::indexerForward, indexer::indexerStop));

        // POV Left: Eject from shooter (1 second reverse)
        // driver.povLeft().onTrue(ShooterCommands.eject(shooter, 1.0));

        // POV Down: Eject from indexer (1 second reverse)
        // driver.povDown().onTrue(IndexerCommands.eject(indexer, 1.0));

        // Full shoot sequence
        // driver.rightTrigger(0.5).whileTrue(
        //     ShooterCommands.shootSequence(shooter, indexer,
        //         ShooterSubsystem.CLOSE_SHOT_RPM, ShooterSubsystem.CLOSE_SHOT_HOOD)
        // );

        // Climber controls
        // driver.povUp().onTrue(climber.extendArm());
        // driver.povDown().onTrue(climber.retractArm());
        // driver.povRight().whileTrue(climber.extendArm());
        // driver.povLeft().whileTrue(climber.retractArm());

 // POV Up/Down: Incremental hood angle adjustment
        // driver.povUp().onTrue(ShooterCommands.increaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // driver.povDown().onTrue(ShooterCommands.decreaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));



        // POV Up/Down: Slow manual drive for alignment
        // driver.povUp().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        // driver.povDown().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // B: Point wheels at joystick direction
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // =====================================================================
        // OPERATOR CONTROLLER (Port 1) - Commented out (TODO: enable as needed)
        // =====================================================================

        // Right Trigger: Vision-based shot
        // operator.rightTrigger(0.5).whileTrue(ShooterCommands.visionShot(shooter, vision));

        // Right Bumper: Conveyor forward
        // operator.rightBumper().whileTrue(
        //     Commands.startEnd(() -> indexer.conveyorForward(), () -> indexer.conveyorStop(), indexer));

        // POV Up/Down: Incremental hood angle adjustment
        // operator.povUp().onTrue(ShooterCommands.increaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // operator.povDown().onTrue(ShooterCommands.decreaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));

        // POV Up/Down: Flywheel velocity adjustment
        // operator.povUp().onTrue(ShooterCommands.increaseTargetVelocity(shooter, ShooterSubsystem.FLYWHEEL_TEST_INCREMENT_RPM));
        // operator.povDown().onTrue(ShooterCommands.decreaseTargetVelocity(shooter, ShooterSubsystem.FLYWHEEL_TEST_INCREMENT_RPM));

        // Indexer testing
        // operator.a().whileTrue(Commands.startEnd(indexer::conveyorForward, indexer::conveyorStop));
        // operator.b().whileTrue(Commands.startEnd(indexer::conveyorReverse, indexer::conveyorStop));
        // operator.x().whileTrue(Commands.startEnd(indexer::indexerForward, indexer::indexerStop));
        // operator.y().whileTrue(Commands.startEnd(indexer::indexerReverse, indexer::indexerStop));

        // Stop climber
        // operator.start().onTrue(climber.stopClimber());
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

} // End of