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
import frc.robot.commands.FarShotCommand;
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
    // Operator controller removed for single-controller testing

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
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, indexer, intake, shooter, fuelCommands);
        SmartDashboard.putData(autoChooser);
        autoChooser.addRoutine("Four meters", autoRoutines::FM);
        autoChooser.addRoutine("Lob", autoRoutines::Lob);
        autoChooser.addRoutine("StartRight goes to middle", autoRoutines::StartRMid);
        autoChooser.addRoutine("Test(center shot)", autoRoutines::TestRoutine);

        SmartDashboard.putData("AutoChooser", autoChooser);
        // Keep flywheel spinning at STANDBY_RPM when no other command is running
        shooter.setDefaultCommand(Commands.run(shooter::returnToStandby, shooter));

        configureBindings();
    }

    private void configureBindings() {
        // =====================================================================
        // DRIVING - Left stick translate, right stick rotate
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
        // INTAKE
        // LT: Intake fuel
        // RB: Retract intake
        // =====================================================================

        driver.leftTrigger().whileTrue(intake.intakeFuel());
        driver.leftBumper().whileTrue(intake.compressFuelIncremental());

        // =====================================================================
        // SHOOTER
        // RT: Shoot with selected preset
        // =====================================================================

        // RT: Shoot — routes to FarShotCommand if FAR preset is selected, otherwise uses selected preset
        driver.rightTrigger(0.5).whileTrue(
            Commands.either(
                new FarShotCommand(shooter, indexer, drivetrain,
                    () -> -driver.getLeftY() * MaxSpeed,
                    () -> -driver.getLeftX() * MaxSpeed),
                FuelCommands.shootWithSelectedPreset(shooter, indexer),
                shooter::isFarShotSelected
            )
        );

        // =====================================================================
        // PRESETS
        // A: Close Shot
        // X: Far Shot (pose-based, auto-rotates to hub, activates on RT)
        // B: Passing Shot (command TBD)
        // =====================================================================

        driver.a().onTrue(Commands.runOnce(() -> shooter.selectPreset(ShooterSubsystem.ShotPreset.CLOSE)));
        driver.x().onTrue(Commands.runOnce(() -> shooter.selectPreset(ShooterSubsystem.ShotPreset.FAR)));
        driver.b().onTrue(Commands.runOnce(() -> shooter.selectPreset(ShooterSubsystem.ShotPreset.PASS)));    // TODO: Replace with Passing Shot preset

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