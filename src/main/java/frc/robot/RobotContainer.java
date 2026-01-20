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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AutoCommands;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.VisionIO;
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

    // ===== Controllers =====
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // ===== Subsystems =====
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision;
    public final IntakeSubsystem intake;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    // ===== Shuffleboard =====
    private final ShuffleboardTab testTab;

    public RobotContainer() {
        // Instantiate subsystems based on robot mode
        // TODO: Change to Robot.isReal() for actual robot
        boolean useRealHardware = false;  // Set to true when deploying to real robot

        if (useRealHardware) {
            // Real hardware
            vision = new VisionSubsystem(new VisionIOLimelight("limelight"));
            intake = new IntakeSubsystem(new IntakeIOTalonFX());
            indexer = new IndexerSubsystem(new IndexerIOTalonFX());
            shooter = new ShooterSubsystem(new ShooterIOTalonFX());
        } else {
            // Simulation
            vision = new VisionSubsystem(new VisionIOSim());
            intake = new IntakeSubsystem(new IntakeIOSim());
            indexer = new IndexerSubsystem(new IndexerIOSim());
            shooter = new ShooterSubsystem(new ShooterIOSim());
        }

        // Setup auto
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Setup Shuffleboard test tab
        testTab = Shuffleboard.getTab("Test");
        setupShuffleboard();

        // Configure controller bindings
        configureBindings();
    }

    private void configureBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    /**
     * Configure driver controller bindings.
     * Driver focuses on drivetrain control.
     */
    private void configureDriverBindings() {
        // Default drive command
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // A - Brake (X formation)
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // B - Point wheels at stick
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // POV Up/Down - Precise forward/backward
        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Left Bumper - Reset field-centric heading
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Right Bumper - Intake sequence (driver assist)
        driver.rightBumper().whileTrue(
            AutoCommands.intakeSequence(intake, indexer)
        );

        // SysId routines when holding back/start and X/Y
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Configure operator controller bindings.
     * Operator focuses on game piece manipulation.
     */
    private void configureOperatorBindings() {
        // ===== Intake Controls =====
        // Left Bumper - Start intaking
        operator.leftBumper().whileTrue(
            IntakeCommands.startIntaking(intake)
        ).onFalse(
            IntakeCommands.retract(intake)
        );

        // Left Trigger - Eject from intake
        operator.leftTrigger(0.5).whileTrue(
            IntakeCommands.eject(intake)
        );

        // ===== Indexer Controls =====
        // Right Bumper - Manual feed to shooter
        operator.rightBumper().whileTrue(
            IndexerCommands.feed(indexer)
        ).onFalse(
            IndexerCommands.idle(indexer)
        );

        // Right Trigger - Eject from indexer
        operator.rightTrigger(0.5).whileTrue(
            IndexerCommands.eject(indexer)
        );

        // ===== Shooter Presets =====
        // A - Close shot
        operator.a().onTrue(
            ShooterCommands.closeShot(shooter)
        );

        // B - Far shot
        operator.b().onTrue(
            ShooterCommands.farShot(shooter)
        );

        // X - Pass
        operator.x().onTrue(
            ShooterCommands.pass(shooter)
        );

        // Y - Vision shot (spin up only)
        operator.y().onTrue(
            ShooterCommands.visionShot(shooter, vision)
        );

        // ===== Complete Sequences =====
        // POV Up - Vision shoot sequence (complete)
        operator.povUp().onTrue(
            AutoCommands.visionShootSequence(shooter, vision, indexer)
        );

        // POV Right - Close shoot sequence (complete)
        operator.povRight().onTrue(
            AutoCommands.closeShootSequence(shooter, indexer)
        );

        // POV Down - Eject all
        operator.povDown().onTrue(
            AutoCommands.ejectAll(intake, indexer, shooter, 1.0)
        );

        // POV Left - Idle all (safe state)
        operator.povLeft().onTrue(
            AutoCommands.idleAll(intake, indexer, shooter)
        );

        // Start - Shooter idle
        operator.start().onTrue(
            ShooterCommands.idle(shooter)
        );

        // Back - Warm up shooter
        operator.back().onTrue(
            ShooterCommands.warmUp(shooter)
        );
    }

    /**
     * Setup Shuffleboard test tab with virtual buttons.
     */
    private void setupShuffleboard() {
        // ===== Intake Test Buttons =====
        testTab.add("Start Intake", IntakeCommands.startIntaking(intake))
            .withPosition(0, 0)
            .withSize(2, 1);

        testTab.add("Retract Intake", IntakeCommands.retract(intake))
            .withPosition(0, 1)
            .withSize(2, 1);

        testTab.add("Eject Intake", IntakeCommands.ejectTimed(intake, 1.0))
            .withPosition(0, 2)
            .withSize(2, 1);

        testTab.add("Zero Intake Slide", IntakeCommands.zeroSlide(intake))
            .withPosition(0, 3)
            .withSize(2, 1);

        // ===== Indexer Test Buttons =====
        testTab.add("Transport", IndexerCommands.transport(indexer))
            .withPosition(2, 0)
            .withSize(2, 1);

        testTab.add("Feed Shooter", IndexerCommands.feedTimed(indexer, 0.5))
            .withPosition(2, 1)
            .withSize(2, 1);

        testTab.add("Eject Indexer", IndexerCommands.ejectTimed(indexer, 1.0))
            .withPosition(2, 2)
            .withSize(2, 1);

        testTab.add("Indexer Idle", IndexerCommands.idle(indexer))
            .withPosition(2, 3)
            .withSize(2, 1);

        // ===== Shooter Test Buttons =====
        testTab.add("Close Shot", ShooterCommands.closeShot(shooter))
            .withPosition(4, 0)
            .withSize(2, 1);

        testTab.add("Far Shot", ShooterCommands.farShot(shooter))
            .withPosition(4, 1)
            .withSize(2, 1);

        testTab.add("Vision Shot", ShooterCommands.visionShot(shooter, vision))
            .withPosition(4, 2)
            .withSize(2, 1);

        testTab.add("Shooter Idle", ShooterCommands.idle(shooter))
            .withPosition(4, 3)
            .withSize(2, 1);

        // ===== Complete Sequences =====
        testTab.add("Intake Sequence", AutoCommands.intakeSequence(intake, indexer))
            .withPosition(6, 0)
            .withSize(2, 1);

        testTab.add("Vision Shoot", AutoCommands.visionShootSequence(shooter, vision, indexer))
            .withPosition(6, 1)
            .withSize(2, 1);

        testTab.add("Close Shoot", AutoCommands.closeShootSequence(shooter, indexer))
            .withPosition(6, 2)
            .withSize(2, 1);

        testTab.add("Eject All", AutoCommands.ejectAll(intake, indexer, shooter, 1.0))
            .withPosition(6, 3)
            .withSize(2, 1);

        testTab.add("Idle All", AutoCommands.idleAll(intake, indexer, shooter))
            .withPosition(8, 0)
            .withSize(2, 1);

        testTab.add("Warmup", AutoCommands.warmupSequence(shooter))
            .withPosition(8, 1)
            .withSize(2, 1);

        // ===== Status Indicators =====
        testTab.addBoolean("Has Game Piece", indexer::hasGamePiece)
            .withPosition(8, 2)
            .withSize(2, 1);

        testTab.addBoolean("Shooter Ready", shooter::isReady)
            .withPosition(8, 3)
            .withSize(2, 1);

        testTab.addBoolean("Vision Has Target", vision::hasTarget)
            .withPosition(10, 0)
            .withSize(2, 1);

        testTab.addBoolean("Intake Jammed", intake::isJammed)
            .withPosition(10, 1)
            .withSize(2, 1);

        testTab.addString("Intake State", () -> intake.getState().toString())
            .withPosition(10, 2)
            .withSize(2, 1);

        testTab.addString("Indexer State", () -> indexer.getState().toString())
            .withPosition(10, 3)
            .withSize(2, 1);
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
