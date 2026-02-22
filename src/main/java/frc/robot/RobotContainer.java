// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.TunerConstants;
import frc.robot.commands.AlignToHubCommand;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerIOHardware;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;

@SuppressWarnings("unused")

public class RobotContainer {

    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final GameDataTelemetry gameDataTelemetry = new GameDataTelemetry();

    // ===== Controllers =====
    private final CommandXboxController driver   = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // ===== Subsystems =====
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem  intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem  vision;
    private final LedSubsystem     ledSubsystem;
    // private final ClimberSubsystem climber;

    // Kept as a field so Robot.java can call updateSimVision() every sim tick.
    // Null when running on real hardware.
    private final VisionIOSim visionIOSim;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        intake  = new IntakeSubsystem(new IntakeIOHardware());
        indexer = new IndexerSubsystem(new IndexerIOHardware());
        shooter = new ShooterSubsystem(new ShooterIOHardware());

        // Automatically swap between sim and real Limelight.
        // visionIOSim is only non-null in simulation so Robot.java can update it.
        if (RobotBase.isSimulation()) {
            visionIOSim = new VisionIOSim();
        } else {
            visionIOSim = null;
        }

        VisionIO visionIO = (visionIOSim != null)
                ? visionIOSim
                : new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME);

        vision = new VisionSubsystem(
            visionIO,
            drivetrain::addVisionMeasurement,
            () -> drivetrain.getState().Pose,
            () -> drivetrain.getState().RawHeading.getDegrees()
        );

        ledSubsystem = new LedSubsystem();
        // climber = new ClimberSubsystem();

        autoFactory = drivetrain.createAutoFactory();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    // -------------------------------------------------------------------------
    // Sim update — call from Robot.simulationPeriodic()
    // -------------------------------------------------------------------------

    public void updateSimVision() {
        if (visionIOSim != null) {
            visionIOSim.setSimRobotPose(drivetrain.getState().Pose);
        }
    }

    // -------------------------------------------------------------------------
    // Bindings
    // -------------------------------------------------------------------------

    private void configureBindings() {
        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Drivetrain
        // =====================================================================

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Start: Reset field-centric heading
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Vision Alignment
        // =====================================================================

        // POV Left: Align to hub, interpolate hood/RPM by distance, and shoot.
        driver.povLeft().whileTrue(
            new AlignToHubCommand(drivetrain, shooter, indexer, 1.5)
        );

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shooter Presets
        // =====================================================================

        // A: Arm close shot
        driver.a().onTrue(ShooterCommands.armCloseShot(shooter));

        // B: Arm pass shot
        driver.b().onTrue(ShooterCommands.armPassShot(shooter));

        // X: Arm hub shot — sets isHubShotArmed, spins up to hub defaults,
        //    unlocks the RT → AlignToHubCommand toggle below.
        driver.x().onTrue(new InstantCommand(shooter::setHubShotPreset, shooter));

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shoot
        // =====================================================================

        Trigger hubShotArmed = new Trigger(shooter::isHubShotArmed);

        // RT (hub shot armed): Toggle full align + shoot sequence.
        driver.rightTrigger(0.5).and(hubShotArmed).toggleOnTrue(
            new AlignToHubCommand(drivetrain, shooter, indexer, 1.5)
        );

        // RT (standard shot): Close or pass shot at currently armed preset.
        driver.rightTrigger(0.5).and(hubShotArmed.negate()).whileTrue(
            ShooterCommands.shootAtCurrentTarget(shooter, indexer)
        );

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Intake
        // =====================================================================

        // Left Trigger: Extend slides and run roller while held.
        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // Left Bumper: Retract intake and slowly reverse roller while held.
        driver.leftBumper().whileTrue(intake.retractAndReverseRollerCommand());

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Commented out (TODO: enable as needed)
        // =====================================================================

        // Right Bumper: (free)
        // driver.rightBumper()...

        // Y: Indexer forward while held
        // driver.y().whileTrue(Commands.startEnd(indexer::indexerForward, indexer::indexerStop));

        // POV Up/Down: Incremental hood angle adjustment
        // driver.povUp().onTrue(ShooterCommands.increaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // driver.povDown().onTrue(ShooterCommands.decreaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));

        // Eject fuel from intake
        // driver.y().whileTrue(intake.ejectFuel());

        // =====================================================================
        // OPERATOR CONTROLLER (Port 1) - Commented out (TODO: enable as needed)
        // =====================================================================

        // POV Up/Down: Incremental hood angle adjustment
        // operator.povUp().onTrue(ShooterCommands.increaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // operator.povDown().onTrue(ShooterCommands.decreaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));

        // Indexer testing
        // operator.a().whileTrue(Commands.startEnd(indexer::conveyorForward, indexer::conveyorStop));
        // operator.b().whileTrue(Commands.startEnd(indexer::conveyorReverse, indexer::conveyorStop));
        // operator.x().whileTrue(Commands.startEnd(indexer::indexerForward, indexer::indexerStop));
        // operator.y().whileTrue(Commands.startEnd(indexer::indexerReverse, indexer::indexerStop));

        // Stop climber
        // operator.start().onTrue(climber.stopClimber());
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public void updateGameData() {
        gameDataTelemetry.update();
    }

    public GameDataTelemetry getGameDataTelemetry() {
        return gameDataTelemetry;
    }

}