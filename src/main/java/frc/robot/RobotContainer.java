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
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FuelCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerIOHardware;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeSubsystem;
// import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
    // private final LedSubsystem ledSub;
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        intake = new IntakeSubsystem(new IntakeIOHardware());
        indexer = new IndexerSubsystem(new IndexerIOHardware());
        shooter = new ShooterSubsystem(new ShooterIOHardware());
        vision = new VisionSubsystem(new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME));
        // ledSub = new LedSubsystem(shooter);

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory,drivetrain,indexer, intake, shooter);
        SmartDashboard.putData("AutoChooser", autoChooser);

        // autoChooser.addRoutine("L Trench-Mid-Trench", autoRoutines::LtTrench_Mid_Trench);
        // autoChooser.addRoutine("R Trench-Mid-Trench", autoRoutines::RtTrench_RtMid_RtTrench);
        // autoChooser.addRoutine("R Trench-Mid-Ramp", autoRoutines::RtTrench_Mid_Ramp);
        // autoChooser.addRoutine("R Trench-Mid-Ramp", autoRoutines::RtTrench_Mid_Ramp);
        autoChooser.addRoutine("Center", autoRoutines::Center);
        
        configureBindings();

    }

    private void configureBindings() {

        // ====================
        // DRIVER CONTROLLER
        // ====================

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getLeftY() * MaxSpeed) // FIXME
                    .withVelocityY(driver.getLeftX() * MaxSpeed) // FIXME
                    .withRotationalRate(driver.getRightX() * MaxAngularRate) // FIXME
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
        
        driver.rightBumper().whileTrue(intake.retractSlidesSlowHeldCmd());

        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());
        // Press once to partially retract slides
        driver.leftBumper().onTrue(intake.retractSlidesIncrementalCmd());

        driver.povLeft().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.CLOSE));
        driver.povRight().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TRENCH));
        driver.povUp().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.FAR));
        driver.povDown().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TOWER));

        // ====================
        // OPERATOR CONTROLLER
        // ====================
        // var anyPresetHeld = operator.a().or(operator.b()).or(operator.x()).or(operator.y()); 
        
        operator.a().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TRENCH));
        operator.b().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.CLOSE));
        operator.x().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.TOWER));
        operator.y().whileTrue(
            FuelCommands.shootWithPreset(shooter, indexer, ShooterSubsystem.ShotPreset.FAR));


        // Hold to back a ball out of the chute if it entered prematurely
        operator.rightTrigger(.5).whileTrue(indexer.reverse());
        operator.leftTrigger(0.5).whileTrue(intake.intakeFuel());
;


        // Auto-reverse: intake running + shooter idle + ball detected in chute = premature ball → reverse indexer
        // Sensor in new place so this probably NOT valid anymore
        // new Trigger(() ->
        //     intake.isRollerRunning() &&
        //     shooter.getState() == ShooterSubsystem.ShooterState.IDLE &&
        //     indexer.isFuelDetected()
        // ).whileTrue(indexer.reverse());

        operator.rightBumper().whileTrue(intake.retractSlidesSlowHeldCmd());
        // operator.leftBumper().whileTrue(intake.retractSlidesStack());

        // Back (View ⧉): Reset odometry to botpose — use when robot rides up on a ball
        operator.back().onTrue(drivetrain.resetPoseFromVisionCommand());
    
        operator.povUp().whileTrue(intake.manualSlideExtendHoldCmd());
        operator.povDown().whileTrue(intake.manualSlideRetractHoldCmd());

        // // POV cycles through LED animations (for testing / manual override)
        // operator.povUp().onTrue(ledSub.cycleNext());
        // operator.povDown().onTrue(ledSub.cyclePrev());
// =================================
// LED STATE TRIGGERS
// =================================

    // Shooting — any shoot preset (driver RT, driver POV left, operator A/B/X/Y)
    // Trigger anyShootHeld = driver.rightTrigger(0.5)
    //     .or(driver.povLeft())
    //     .or(operator.a())
    //     .or(operator.b())
    //     .or(operator.x())
    //     .or(operator.y());
    //         anyShootHeld
    //             .onTrue(ledSub.showShooting())
    //             .and(RobotModeTriggers.teleop()).onFalse(ledSub.showIdle());


    // // Intaking — driver or operator left trigger
    // Trigger anyIntakeHeld = driver.leftTrigger(0.5)
    //     .or(operator.leftTrigger(0.5));
    //         anyIntakeHeld
    //             .onTrue(ledSub.showIntaking())
    //             .and(RobotModeTriggers.teleop()).onFalse(ledSub.showIdle());

    // // Default to idle when enabled and nothing else is active
    // RobotModeTriggers.teleop()
    //     .onTrue(ledSub.showIdle());
        // =====================================================================
        // LED GAME TELEMETRY TRIGGERS (commented out — enable when needed)
        // Requires: gameDataTelemetry accessible here, DriverStation import
        // =====================================================================

        // -- Robot alliance color on enable --
        // new Trigger(() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red)
        //     .onTrue(ledSub.showDefault()); // swap showDefault() for a showAllianceRed() if you add one

        // -- Active hub: which alliance is currently in scoring mode --
        // new Trigger(gameDataTelemetry::isRedHubActive)
        //     .onTrue(/* ledSub.showRedHub() */null)
        //     .onFalse(/* ledSub.showDefault() */null);

        // new Trigger(gameDataTelemetry::isBlueHubActive)
        //     .onTrue(/* ledSub.showBlueHub() */null)
        //     .onFalse(/* ledSub.showDefault() */null);

        // -- FMS data received (lights up once auto-scoring data arrives ~3s after auto) --
        // new Trigger(gameDataTelemetry::isDataReceived)
        //     .onTrue(/* ledSub.showAllianceColor() */null);

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
