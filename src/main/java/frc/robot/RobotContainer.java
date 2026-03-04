// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.FuelCommands;
import frc.robot.commands.VisionShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerIOHardware;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;

@SuppressWarnings("unused") // Suppress warnings for unused right now

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
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // ===== Subsystems =====
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    // private final LedSubsystem ledSubsystem;
    private final LedSubsystem larson;
    // private final ClimberSubsystem climber;
    private final FuelCommands fuelCommands = null;
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        intake = new IntakeSubsystem(new IntakeIOHardware());
        indexer = new IndexerSubsystem(new IndexerIOHardware());
        shooter = new ShooterSubsystem(new ShooterIOHardware());
        vision = new VisionSubsystem(new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME));

        larson = new LedSubsystem();
        // climber = new ClimberSubsystem();

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory,drivetrain,indexer, intake, shooter, fuelCommands);
        SmartDashboard.putData(autoChooser);
        autoChooser.addRoutine("Four meters", autoRoutines::FM);
        autoChooser.addRoutine("Lob", autoRoutines::Lob);
        autoChooser.addRoutine("StartRight goes to middle", autoRoutines::StartRMid);
        autoChooser.addRoutine("Test(center shot)", autoRoutines::TestRoutine);
        autoChooser.addRoutine("Test(Right shot)", autoRoutines::TestRoutineR);
        
        SmartDashboard.putData("AutoChooser", autoChooser);
        configureBindings();

    }

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
        // DRIVER CONTROLLER (Port 0) - Shooter
        // =====================================================================

        // Operator holds a face button to override with a named preset.
        // Driver RT alone fires VisionShootCommand (default — auto-aims by distance).
        var anyPresetHeld = operator.a().or(operator.b()).or(operator.x()).or(operator.y());

        driver.rightTrigger(0.5).and(operator.a()).whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TRENCH));
        driver.rightTrigger(0.5).and(operator.b()).whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.CLOSE));
        driver.rightTrigger(0.5).and(operator.x()).whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TOWER));
        driver.rightTrigger(0.5).and(operator.y()).whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.FAR));

        driver.rightTrigger(0.5).and(anyPresetHeld.negate()).whileTrue(
            new VisionShootCommand(shooter, indexer, drivetrain,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed));

        driver.rightBumper().whileTrue(FuelCommands.shootPass(shooter, indexer));

        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());
        driver.leftBumper().whileTrue(intake.compressFuelIncremental());

        // =====================================================================
        // OPERATOR CONTROLLER (Port 1)
        // =====================================================================

        operator.leftTrigger().whileTrue(FuelCommands.runAirPopper(indexer, shooter, intake));
        // TODO: Test the Command retractSlidesWithRollerCmd() from IntakeSubSystem
        operator.leftBumper().whileTrue(intake.retractSlidesWithRollerCmd());

        // Show preset label on Elastic while operator holds the button (no trigger required).
        // Reverts to "Vision" when the button is released.
        operator.a().onTrue(Commands.runOnce(() -> shooter.setDisplayPreset(ShooterSubsystem.ShotPreset.TRENCH)));
        operator.a().onFalse(Commands.runOnce(shooter::clearDisplayPreset));
        operator.b().onTrue(Commands.runOnce(() -> shooter.setDisplayPreset(ShooterSubsystem.ShotPreset.CLOSE)));
        operator.b().onFalse(Commands.runOnce(shooter::clearDisplayPreset));
        operator.x().onTrue(Commands.runOnce(() -> shooter.setDisplayPreset(ShooterSubsystem.ShotPreset.TOWER)));
        operator.x().onFalse(Commands.runOnce(shooter::clearDisplayPreset));
        operator.y().onTrue(Commands.runOnce(() -> shooter.setDisplayPreset(ShooterSubsystem.ShotPreset.FAR)));
        operator.y().onFalse(Commands.runOnce(shooter::clearDisplayPreset));

        // operator.povUp().whileTrue(null); // incremental extend climber command to be added when climber is ready
        // operator.povDown().whileTrue(null); // incremental retract climber command to be added when climber is ready

        // TODO: Test this new shoot + retract command and tune the slide retract time
        operator.rightTrigger().whileTrue(FuelCommands.Auto.shootTrenchWithSlideRetract(shooter, indexer, intake, 3));
                
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

} // End of Class RobotContainer