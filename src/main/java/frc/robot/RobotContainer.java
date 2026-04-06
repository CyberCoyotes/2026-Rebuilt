// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.FuelCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerIOHardware;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    // =====================================================================
    // Drive Tuning
    // =====================================================================
    private double MaxSpeed = Constants.DRIVE_CLAMP * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = Constants.DRIVE_CLAMP * RotationsPerSecond.of(1.0).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // Deadband of 15% on translation and rotation inputs to prevent unintended movement when joysticks are near their centers. 
            // Adjust as needed based on driver feedback.
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final GameDataTelemetry gameDataTelemetry = new GameDataTelemetry();

    // =====================================================================
    // Controllers
    // =====================================================================
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // =====================================================================
    // Subsystems
    // =====================================================================
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    // =====================================================================
    // Constructor
    // =====================================================================
    public RobotContainer() {
        intake = new IntakeSubsystem(new IntakeIOHardware());
        indexer = new IndexerSubsystem(new IndexerIOHardware());
        shooter = new ShooterSubsystem(new ShooterIOHardware());
        vision = new VisionSubsystem(new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME));

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory,drivetrain,indexer, intake, shooter);
        SmartDashboard.putData("AutoChooser", autoChooser);

        // =====================================================================
        // Verified autoRoutines to chooser
        // =====================================================================

        // autoChooser.addRoutine("L Trench-Mid-Trench", autoRoutines::LtTrench_Mid_Trench);
        // autoChooser.addRoutine("R Trench-Mid-Trench", autoRoutines::RtTrench_RtMid_RtTrench);
        // autoChooser.addRoutine("R Trench-Mid-Ramp", autoRoutines::RtTrench_Mid_Ramp);
        // autoChooser.addRoutine("R Trench-Mid-Ramp", autoRoutines::RtTrench_Mid_Ramp);
        autoChooser.addRoutine("Center", autoRoutines::Center);
        
        configureBindings();

    }

    private void configureBindings() {

        // =====================================================================
        // Driver Controller
        // =====================================================================

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getLeftY() * MaxSpeed)
                    .withVelocityY(driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(driver.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Start: Reset field-centric heading
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Back: Reset odometry to Limelight botpose (use when robot rides up on a ball and wheels lose contact)
        driver.back().onTrue(drivetrain.resetPoseFromVisionCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
        
        driver.rightTrigger(0.5).whileTrue(
            FuelCommands.poseAlignAndShoot(shooter, indexer, /*intake,*/ drivetrain,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed)); 

        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());
        driver.a().onTrue(intake.extendSlidesFastCmd());
        driver.b().onTrue(intake.retractSlidesFastCmd());
        driver.x().onTrue(intake.fuelCompression());
        driver.y().whileTrue(intake.fuelPumpCycleDelayed());
        driver.leftBumper().onTrue(intake.retractSlidesIncrementalCmd());

        driver.povLeft().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.CLOSE));
        driver.povRight().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TRENCH));
        driver.povUp().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.FAR));
        driver.povDown().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TOWER));

        // =====================================================================
        // Operator Controller
        // =====================================================================
        operator.a().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TRENCH));
        operator.b().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.CLOSE));
        operator.x().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TOWER));
        operator.y().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.FAR));

        operator.rightTrigger(0.5).whileTrue(indexer.reverse());
        operator.leftTrigger(0.5).whileTrue(intake.intakeFuel());
        operator.rightBumper().onTrue(intake.retractSlidesFastCmd());
        operator.leftBumper().onTrue(intake.fuelCompression());

        // Back (View ⧉): Reset odometry to botpose — use when robot rides up on a ball
        operator.back().onTrue(drivetrain.resetPoseFromVisionCommand());
    
        operator.povUp().whileTrue(intake.manualSlideExtendHoldCmd());
        operator.povDown().whileTrue(intake.manualSlideRetractHoldCmd());
        operator.povLeft().onTrue(intake.extendSlidesFastCmd());
        operator.povRight().whileTrue(intake.fuelPumpCycleDelayed());

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
