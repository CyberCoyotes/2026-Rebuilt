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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.AlignToHubCommand;
import frc.robot.commands.FarShotCommand;
import frc.robot.commands.ShooterCommands;
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
    private final LedSubsystem ledSubsystem;
    // private final ClimberSubsystem climber;

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

        // ----- Shooter -----
        // ----- Full Sequences -----
        // A: Full shoot sequence - close shot with indexer feed, then idle
        /*
        driver.rightTrigger(0.5).whileTrue(
            ShooterCommands.shootSequence(shooter, indexer,
                ShooterSubsystem.CLOSE_SHOT_RPM, ShooterSubsystem.CLOSE_SHOT_HOOD)
        );
         */

        // Right Trigger: Flywheel ramp-up test — hold to ramp to target RPM, release to idle.
        // Ramp rate governed by ClosedLoopRamps in TalonFXConfigs (currently 4s).
        // driver.rightTrigger(0.5).whileTrue(
        //     Commands.sequence(
        //         shooter.closeShotCommand(),
        //         indexer.feed()
        // ).finallyDo(() -> {
        //     shooter.setIdle();
        //     indexer.stop();
        // })
        // );

        /* TODO Test rampTestShoot first, comment out, and try this one */
        /*
        driver.rightTrigger(0.5).whileTrue(
            Commands.parallel(
                ShooterCommands.rampUpFlywheel(shooter, ShooterSubsystem.RAMP_TEST_TARGET_RPM),
                Commands.waitUntil(shooter::isReady).withTimeout(5.0), // Timeout to prevent indefinite waiting if something goes wrong
                Commands.run(indexer::indexerForward).withTimeout(10.0) // Run indexer forward for 10 seconds or until interrupted (e.g., by releasing trigger)
        // POV Left: Align to blue hub (tags 18-27).
        // Robot rotates to face the nearest hub AprilTag while driver controls translation.
        driver.povLeft().whileTrue(
            new AlignToHubCommand(
                drivetrain,
                vision,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed
            )
        );

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shooter Presets
        // Silently update target RPM and hood angle.
        // No motors move until the shoot trigger is pressed.
        // =====================================================================

        // A: Arm close shot — clears far shot flag, routes RT to shootAtCurrentTarget
        driver.a().onTrue(ShooterCommands.armCloseShot(shooter));

        // B: Pass shot (prepare and wait for ready)
        driver.b().onTrue(Commands.runOnce(() -> {
            shooter.setTargetHoodPose(ShooterSubsystem.PASS_SHOT_HOOD);
            shooter.prepareToShoot();
        }));
        driver.y().whileTrue(
            Commands.startEnd(indexer::indexerForward, indexer::indexerStop));
        // POV Up: Eject from shooter (clear jams, 1 second reverse)
        
        // driver.povLeft().onTrue(ShooterCommands.eject(shooter, 1.0));
        // X: Arm far shot — sets far shot flag, routes RT to FarShotCommand
        driver.x().onTrue(ShooterCommands.armFarShot(shooter));

        // B: Arm pass shot — clears far shot flag, routes RT to shootAtCurrentTarget
        driver.b().onTrue(ShooterCommands.armPassShot(shooter));

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shoot
        // =====================================================================

        // Right Trigger: Shoot at currently armed preset.
        //
        // If FAR SHOT is armed (X was pressed):
        //   → FarShotCommand — auto-rotates to hub, continuously adjusts hood by distance,
        //     RPM at FAR_SHOT_RPM + 350, driver controls translation
        //
        // Otherwise (close shot or pass shot):
        //   → shootAtCurrentTarget — ramps to preset targets, waits until ready, feeds
        //
        // Both commands return to SPINUP at IDLE_RPM on release.

        Trigger rtTrigger = driver.rightTrigger(0.5);
        Trigger farShotArmed = new Trigger(shooter::isFarShotArmed);

        // Far shot path — only when X was last pressed
        rtTrigger.and(farShotArmed).whileTrue(
            new FarShotCommand(
                drivetrain,
                shooter,
                vision,
                indexer,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed
            )
        );

        // Standard shot path — when A or B was last pressed
        rtTrigger.and(farShotArmed.negate()).whileTrue(
            ShooterCommands.shootAtCurrentTarget(shooter, indexer)
        );

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Intake
        // =====================================================================

        // Left Trigger: Extend slides and run roller while held.
        // Slides remain extended on release (intentional).
        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // Left Bumper: Stop intake jam (quick reverse)
        // driver.leftBumper().whileTrue(intake.ejectFuel());
        driver.leftBumper().onTrue(Commands.runOnce(intake::retractSlides, intake));

        // ----- Climber (POV) -----
        // POV Up: Extend climber arm (preset})
        //  driver.povUp().onTrue(climber.extendArm());
        // POV Down: Retract climber arm (Preset)
        // driver.povDown().onTrue(climber.retractArm());
        
        // POV Right: Extend climber arm (while held)
        // driver.povRight().whileTrue(climber.extendArm());
        // POV Left: Retract climber arm (while held)
        // driver.povLeft().whileTrue(climber.retractArm());

        // Start: Stop climber
       // operator.start().onTrue(climber.stopClimber());

        // ----- Operator Controller (Port 1) - Shooter Hood Testing -----
        // operator.povDown().onTrue(shooter.runOnce(shooter::decreaseHoodForTesting));
        // operator.povUp().onTrue(shooter.runOnce(shooter::increaseHoodForTesting));

        // Hood position testing (closed-loop position control)
        // operator.rightBumper().onTrue(shooter.runHoodToMax());   // Move hood to MAX_HOOD_POSE
        // operator.leftBumper().onTrue(shooter.runHoodToMin());    // Move hood to MIN_HOOD_POSE
        operator.rightBumper().whileTrue(
            Commands.startEnd(
                () -> indexer.conveyorForward(), 
                () -> indexer.conveyorStop(), 
                indexer)
        );


        //
        // operator.rightTrigger(0.5).whileTrue(
        //     ShooterCommands.visionShot(shooter, vision)
        // );
        

        // /* */
        // operator.rightTrigger(0.5).whileTrue(
        //     Commands.parallel(
        //         ShooterCommands.rampUpFlywheel(shooter, ShooterSubsystem.RAMP_TEST_TARGET_RPM),
        //         Commands.waitUntil(shooter::isReady).withTimeout(10.0), // Timeout to prevent indefinite waiting if something goes wrong
        //         Commands.run(indexer::indexerForward).withTimeout(10.0) // Run indexer forward for 10 seconds or until interrupted (e.g., by releasing trigger)
        //     )
        // );
    
        // Left Bumper: Retract intake slides while held.
        driver.leftBumper().whileTrue(intake.retractSlidesCommand());

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Commented out (TODO: enable as needed)
        // =====================================================================

        // Right Bumper: (free)
        // driver.rightBumper()...

        // Y: Indexer forward while held
        // driver.y().whileTrue(Commands.startEnd(indexer::indexerForward, indexer::indexerStop));

        // POV Right/Up/Down: (free)

        // POV Up/Down: Incremental hood angle adjustment
        // driver.povUp().onTrue(ShooterCommands.increaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // driver.povDown().onTrue(ShooterCommands.decreaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));

        // Eject fuel from intake
        // driver.y().whileTrue(intake.ejectFuel());

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

} // End of