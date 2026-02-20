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

import frc.robot.commands.SuperCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    /* TODO Clamp max speed and angular rate for testing 
    * Increase to actual desired values once we have tested the robot's responsiveness and handling at lower speeds.
    * This will help prevent runaway situations while tuning.*/
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 0.75 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

   // private final Telemetry logger = new Telemetry(MaxSpeed);
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
            vision = new VisionSubsystem(new VisionIOSim());


        ledSubsystem = new LedSubsystem();
        // climber = new ClimberSubsystem();

        autoFactory = drivetrain.createAutoFactory();
        
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        // AutoChooser automatically publishes to NetworkTables at "AutoChooser"
        // Elastic dashboard can read this directly without SmartDashboard

        configureBindings();
    }

    private void configureBindings() {
        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Drivetrain
        // =====================================================================

        // Default: Field-centric drive with left stick (translate) and right stick (rotate)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
       
        // B: Point wheels at joystick direction (for testing)
       /*  
       driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // POV Up/Down: Slow manual drive (for alignment/testing)
        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        */

        // start: Reset field-centric heading
        // driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        
        //drivetrain.registerTelemetry(logger::telemeterize);

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Mechanisms
        // =====================================================================

        // Testing shooter and indexer together with SuperCommands. Use whileTrue() to automatically stop on release.

        // TODO Test these commands and tune shooter RPMs and hood positions for each shot type. The current values are placeholders.

        // Right Trigger: Close shot
        // driver.rightTrigger(0.5).whileTrue(SuperCommands.closeShot(shooter, indexer));
        
        // Long shot
        // driver.rightTrigger(0.5).whileTrue(SuperCommands.towerShot(shooter, indexer));
        
        // Trench shot
        // driver.rightTrigger().whileTrue(SuperCommands.trenchShot(shooter, indexer));

        // Team Shot
        driver.rightTrigger().whileTrue(SuperCommands.shoot3603(shooter, indexer));


        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shooting
        // Flywheel spins up immediately on press, indexer feeds once ready.
        // Everything stops automatically on button release.
        // =====================================================================

        // Right Trigger: Close shot (subwoofer / near target)
        // driver.rightTrigger(0.5).whileTrue(SuperstructureCommands.closeShot(shooter, indexer));

        // Left Trigger: Long shot (far from target)
        // driver.leftTrigger(0.5).whileTrue(SuperstructureCommands.longShot(shooter, indexer));

        // Right Bumper: Trench shot (tune TRENCH_SHOT_RPM / TRENCH_SHOT_HOOD in ShooterSubsystem)
        // driver.rightBumper().whileTrue(SuperstructureCommands.trenchShot(shooter, indexer));

        // POV Left: Eject from shooter (clear jams)
        // driver.povLeft().whileTrue(shooter.eject());

        // ----- Intake -----
        // Left Trigger: Run intake rotator and slides(while held)
        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // Left Bumper: Stop intake jam (quick reverse)
        driver.leftBumper().whileTrue(intake.retractSlidesCommand());
        // driver.leftBumper().onTrue(Commands.runOnce(intake::retractSlides, intake));

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

        // =====================================================================
        // OPERATOR CONTROLLER (Port 1) - Tuning / Testing
        // =====================================================================

        // POV Up/Down: Bump flywheel RPM +/- 100 for live tuning
        // operator.povUp().onTrue(shooter.increaseVelocity());
        // operator.povDown().onTrue(shooter.decreaseVelocity());

        // POV Right/Left: Bump hood position for live tuning
        // operator.povRight().onTrue(shooter.increaseHood());
        // operator.povLeft().onTrue(shooter.decreaseHood());

        // Individual motor testing (conveyor / indexer)
        // TODO Test
        driver.a().whileTrue(Commands.startEnd(indexer::conveyorForward, indexer::conveyorStop, indexer)); 
        driver.b().whileTrue(Commands.startEnd(indexer::conveyorReverse, indexer::conveyorStop, indexer));
        // operator.x().whileTrue(Commands.startEnd(indexer::indexerForward, indexer::indexerStop, indexer));
        // operator.y().whileTrue(Commands.startEnd(indexer::indexerReverse, indexer::indexerStop, indexer));

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