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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import frc.robot.commands.AlignOnlyCommand;
import frc.robot.commands.FuelCommands;

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

    // private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();


    // =====================================================================
    // Controllers
    // =====================================================================
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // =====================================================================
    // Subsystems
    // =====================================================================
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.SwerveDriveBrake xBrake = new SwerveRequest.SwerveDriveBrake();
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
        indexer = new IndexerSubsystem(new IndexerIOHardware());
        intake = new IntakeSubsystem(new IntakeIOHardware());
        shooter = new ShooterSubsystem(new ShooterIOHardware());
        vision = new VisionSubsystem(
            new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME),
            () -> drivetrain.getState().Pose.getRotation().getDegrees(),
            () -> Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory,drivetrain,indexer, intake, shooter);
        SmartDashboard.putData("AutoChooser", autoChooser);

        // =====================================================================
        // Verified autoRoutines to chooser
        // =====================================================================

        // == Left Side Autos ==
        autoChooser.addRoutine("Left x1 Ramp ANGRY", autoRoutines::LtTrench_Ramp_Single);
        autoChooser.addRoutine("Left x2 Ramp ANGRY", autoRoutines::LtTrench_Ramp_Double);
        autoChooser.addRoutine("Left x2 Ramp Sweep SHOT", autoRoutines::LtTrench_Ramp_HubSweep);
        // autoChooser.addRoutine("Left Ramp Sweep PURGE", autoRoutines::LtTrench_Ramp_Sweep_Purge);
        // autoChooser.addRoutine("Left Ramp Sweep PURGE", autoRoutines::LtTrench_Ramp_Sweep_AngryPurge);
        
        // == Right Side Autos ==
        autoChooser.addRoutine("Right x1 Wait Ramp ANGRY", autoRoutines::RtTrench_Ramp_Single);
        autoChooser.addRoutine("Right x1 Ramp MEEP", autoRoutines::RtTrench_Ramp_Meep);
        autoChooser.addRoutine("Right x2 Ramp ANGRY", autoRoutines::RtTrench_Ramp_Double);
        autoChooser.addRoutine("Right x2 Ramp Sweep SHOT", autoRoutines::RtTrench_Ramp_HubSweep);
        // autoChooser.addRoutine("Right Ramp Sweep PURGE", autoRoutines::RtTrench_Ramp_Sweep_Purge);
        // autoChooser.addRoutine("Right Angry Meep Meep", autoRoutines::RtTrench_Ramp_AngryMeepMeep);
        // autoChooser.addRoutine("Right Ramp Sweep ANGRY PURGE", autoRoutines::RtTrench_Ramp_Sweep_AngryPurge);

        autoChooser.addRoutine("Right Bulldozer 2026", autoRoutines::RtBulldozer);

        // Testing
        // autoChooser.addRoutine("Rt Bulldozer 2026", autoRoutines.test::Bulldozer);
        autoChooser.addRoutine("Center", autoRoutines::Center);
        
        configureBindings();

    }

    private void configureBindings() {

        // =====================================================================
        // Driver Controller
        // =====================================================================

        // It is critical that these inputs are (-). Do not change them.
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

        // Back: Reset odometry to Limelight botpose (use when robot rides up on a ball and wheels lose contact)
        driver.back().onTrue(drivetrain.resetPoseFromVisionCommand());


        drivetrain.registerTelemetry(logger::telemeterize);
        
        driver.rightTrigger(0.5).whileTrue(
            Commands.deadline(
                FuelCommands.poseAlignAndShoot(
                    shooter,
                    indexer,
                    drivetrain,
                    vision,
                    () -> -driver.getLeftY() * MaxSpeed,
                    () -> -driver.getLeftX() * MaxSpeed
                ),
                /* fuelCompressionWhenShooterReady() */
                intake.fuelCompression()
            )
        );
        driver.rightBumper().whileTrue(
            Commands.deadline(
                // drivetrain.applyRequest(() -> xBrake),    
                FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.CLOSE),
                intake.fuelCompression() // REMOVE for testing
                ));

        // driver.rightBumper().whileTrue(FuelCommands.purgeFuel(intake, indexer));
        
        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());
        // Align-only: rotation + vision logic, no flywheel/hood — safe for PID tuning
        driver.leftBumper().whileTrue(
            new AlignOnlyCommand(
                drivetrain,
                vision,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed
            )
        );
        
        // =====================================================================
        // Operator Controller
        // =====================================================================
        operator.a().whileTrue(
            Commands.deadline(
                // drivetrain.applyRequest(() -> xBrake),
                FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TRENCH),
                intake.fuelCompression()
                ));

        operator.b().whileTrue(
            Commands.deadline(
                // drivetrain.applyRequest(() -> xBrake),    
                FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.CLOSE),
                intake.fuelCompression() // REMOVE for testing
                ));
        operator.x().whileTrue(
            Commands.deadline(
                // drivetrain.applyRequest(() -> xBrake),    
                FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TOWER),
                intake.fuelCompression()
                ));
        operator.y().whileTrue(
            Commands.deadline(
                // drivetrain.applyRequest(() -> xBrake),
                FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.FAR),
                intake.fuelCompression()
                ));

        operator.leftTrigger(0.5).whileTrue(intake.intakeFuel());
        
        // operator.rightTrigger(0.5).whileTrue(Align indexer.reverse());
        // operator.rightTrigger(0.5).whileTrue(indexer.reverse());


        operator.leftBumper().onTrue(intake.retractSlidesIncrementalCmd());
        operator.rightBumper().whileTrue(intake.fuelCompression()
        /*FuelCommands.purgeFuel(intake, indexer)*/);
        

        // Start (Menu ☰): Toggle flywheel standby pre-rev — operator sets once and forgets.
        // When ON: flywheel holds at STANDBY_RPM (1800) between shots instead of stopping.
        // When OFF: flywheel returns to full idle after each shot.
        // Defaults OFF at robot startup — operator must enable explicitly.
        operator.start().onTrue(Commands.runOnce(shooter::toggleStandbyMode, shooter));


        // Back (View ⧉): Reset odometry to botpose — use when robot rides up on a ball
        operator.back().onTrue(drivetrain.resetPoseFromVisionCommand());
    
        operator.povUp().whileTrue(drivetrain.applyRequest(() -> xBrake));
        // operator.povDown().whileTrue(intake.manualSlideRetractHoldCmd());

        // operator.povLeft().onTrue(intake.extendSlidesFastCmd());
        // operator.povRight().whileTrue(intake.fuelPumpCycleDelayed());
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    /**
     * Runs intake fuel compression only after the shooter reports ready.
     * If the shooting command is cancelled before ready, compression never starts.
     */
    private Command fuelCompressionWhenShooterReady() {
        return Commands.sequence(
                Commands.waitUntil(shooter::isReady),
                intake.fuelCompression())
                .withName("FuelCompressionWhenShooterReady");
    }

    public void updateGameData() {
        gameDataTelemetry.update();
    }

    public GameDataTelemetry getGameDataTelemetry() {
        return gameDataTelemetry;
    }

} // End of Class RobotContainer
