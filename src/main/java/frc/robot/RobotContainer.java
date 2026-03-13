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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.FuelCommands;
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
    private final LedSubsystem ledSub;
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

        ledSub = new LedSubsystem();
        // climber = new ClimberSubsystem();

        // NamedCommands.registerCommand("Shoot",
        // ShooterCommands.visionAlignAndShoot(shooter, vision, indexer, drivetrain));

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory,drivetrain,indexer, intake, shooter, fuelCommands, vision/*, ledSubsystem*/);
        SmartDashboard.putData(autoChooser);

        autoChooser.addRoutine("Rt Trench-Mid-Trench", autoRoutines::RtTrench_Mid_Trench);
        autoChooser.addRoutine("Rt Trench-Mid-Ramp", autoRoutines::RtTrench_Mid_Ramp);
        
        autoChooser.addRoutine("Lt Trench-Mid-Trench", autoRoutines::LtTrench_Mid_Trench);
        autoChooser.addRoutine("Rt Trench-Mid-Ramp", autoRoutines::RtTrench_Mid_Ramp);


        autoChooser.addRoutine("Center Depot shot)", autoRoutines:: MidDepot);

        autoChooser.addRoutine("Rt Trench-Mid-Trench (Split)", autoRoutines::RtTrench_Mid_Trench_Splits);

        
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
        
        driver.rightTrigger(0.5).whileTrue(
            FuelCommands.poseAlignAndShoot(shooter, indexer, /*intake,*/ drivetrain,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed)); 
        
        // Hold on and it will cycle back and forth
        driver.rightBumper().whileTrue(intake.fuelPumpCycle());

        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // Prese once it will retract fully
        driver.leftBumper().onTrue(intake.retractSlidesCmd());

        // driver.a().whileTrue(FuelCommands.fuelPump(indexer));
        


        // =====================================================================
        // OPERATOR CONTROLLER (Port 1)
        // =====================================================================
        // var anyPresetHeld = operator.a().or(operator.b()).or(operator.x()).or(operator.y()); 
        
        operator.a().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TRENCH));
        operator.b().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.CLOSE));
        operator.x().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TOWER));
        operator.y().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.FAR));

        operator.rightBumper().whileTrue(FuelCommands.shootPass(shooter, indexer));

        operator.leftTrigger().whileTrue(FuelCommands.runAirPopper(indexer, shooter, intake));
        operator.leftBumper().whileTrue(intake.retractSlidesStack());

    
        // operator.povUp().whileTrue(null); // incremental extend climber command to be added when climber is ready
        // operator.povDown().whileTrue(null); // incremental retract climber command to be added when climber is ready

        // Operator holds a face button to override with a named preset.

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