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
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.climber.ClimberSubsystem;
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
    /* TODO Lower max speed and angular rate for testing, 
    * then increase to actual desired values once we have tested the robot's responsiveness and handling at lower speeds.
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
    // private final IntakeCommands intakeCommands; // Not needed since we can just use intake subsystem methods directly in bindings

    /* Path follower */
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
       /*  driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // POV Up/Down: Slow manual drive (for alignment/testing)
        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );*/
        // start: Reset field-centric heading
        // driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        
        //drivetrain.registerTelemetry(logger::telemeterize);

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Mechanisms
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
driver.rightTrigger(0.5).whileTrue(
    // Write a command sequence that setsTargetVelocity of the flywheel and the indexer motor at the same time but doesn't require a state machine
        Commands.parallel(
            () -> {
                shooter.setTargetVelocity(3600);
                shooter.prepareToShoot();
            },
            () -> {
                indexer.indexerForward();
            }
        ));

    // Commands.startEnd(
    //     () -> {
    //         shooter.setTargetVelocity(3600);
    //         shooter.prepareToShoot();
    //         indexer.indexerForward();
    //         // indexer.conveyorForward();
    //     },
    //     () -> {
    //         // shooter.setIdle();
    //         indexer.indexerStop();
    //         // indexer.conveyorStop();
    //     },
    //     shooter , indexer//*/
    


        // TODO Test Just run the flywheels
        // Commands.run(
            // () -> shooter.setTargetVelocity(ShooterSubsystem.RAMP_TEST_TARGET_RPM)).withTimeout(10.0) // Timeout to prevent indefinite running if something goes wrong;
        
        // );

        /* TODO Test rampTestShoot first, comment out, and try this one */
        /*
        driver.rightTrigger(0.5).whileTrue(
            Commands.parallel(
                ShooterCommands.rampUpFlywheel(shooter, ShooterSubsystem.RAMP_TEST_TARGET_RPM),
                Commands.waitUntil(shooter::isReady).withTimeout(5.0), // Timeout to prevent indefinite waiting if something goes wrong
                Commands.run(indexer::indexerForward).withTimeout(10.0) // Run indexer forward for 10 seconds or until interrupted (e.g., by releasing trigger)
            )
        );
        */

        // Y: Close shot (prepare and wait for ready)
        // driver.a().onTrue(ShooterCommands.closeShot(shooter)); // TODO Comment out for testing without flywheel
        // Close shot hood preset — sets target AND transitions to READY so motor actually moves
        // driver.a().onTrue(Commands.runOnce(() -> {
        //     shooter.setTargetHoodPose(ShooterSubsystem.CLOSE_SHOT_HOOD);
        //     shooter.prepareToShoot();
        // }));

        // X: Far shot (prepare and wait for ready)
        // driver.x().onTrue(Commands.runOnce(() -> {
        //     shooter.setTargetHoodPose(ShooterSubsystem.FAR_SHOT_HOOD);
        //     shooter.prepareToShoot();
        // }));

        // B: Pass shot (prepare and wait for ready)
        // driver.b().onTrue(Commands.runOnce(() -> {
        //     shooter.setTargetHoodPose(ShooterSubsystem.PASS_SHOT_HOOD);
        //     shooter.prepareToShoot();
        // }));
        // driver.y().whileTrue(
        //     Commands.startEnd(indexer::indexerForward, indexer::indexerStop));
        // // POV Up: Eject from shooter (clear jams, 1 second reverse)
        
        // driver.povLeft().onTrue(ShooterCommands.eject(shooter, 1.0));

        // ----- Indexer -----
        // Right Bumper: Feed game piece to shooter (while held)
        //driver.rightBumper().whileTrue(IndexerCommands.feed(indexer));

        // POV Down: Eject from indexer (clear jams, 1 second reverse)
       // driver.povDown().onTrue(IndexerCommands.eject(indexer, 1.0));

        // ----- Intake -----
        // Left Trigger: Run intake rotator and slides(while held)
        // driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // // Left Bumper: Stop intake jam (quick reverse)
        // // driver.leftBumper().whileTrue(intake.ejectFuel());
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

        // ----- Operator Controller (Port 1) - Shooter Hood Testing -----
        // operator.povDown().onTrue(shooter.runOnce(shooter::decreaseHoodForTesting));
        // operator.povUp().onTrue(shooter.runOnce(shooter::increaseHoodForTesting));

        // Hood position testing (closed-loop position control)
        // operator.rightBumper().onTrue(shooter.runHoodToMax());   // Move hood to MAX_HOOD_POSE
        // operator.leftBumper().onTrue(shooter.runHoodToMin());    // Move hood to MIN_HOOD_POSE
        // operator.rightBumper().whileTrue(
        //     Commands.startEnd(
        //         () -> indexer.conveyorForward(), 
        //         () -> indexer.conveyorStop(), 
        //         indexer)
        // );


        // //
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
    

        // =====================================================================
        // OPERATOR CONTROLLER (Port 1) - Flywheel Velocity Adjustment
        // =====================================================================
        // TODO: Uncomment these bindings when ready for on-robot testing
        // POV Up: Increase flywheel velocity by 100 RPM
        // operator.povUp().onTrue(ShooterCommands.increaseTargetVelocity(shooter, ShooterSubsystem.FLYWHEEL_TEST_INCREMENT_RPM));
        // POV Down: Decrease flywheel velocity by 100 RPM
        // operator.povDown().onTrue(ShooterCommands.decreaseTargetVelocity(shooter, ShooterSubsystem.FLYWHEEL_TEST_INCREMENT_RPM));

        // TODO Add testing bindings for conveyor and indexer
        // operator.a().whileTrue(
        //     Commands.startEnd(indexer::conveyorForward, indexer::conveyorStop));
        // operator.b().whileTrue(
        //     Commands.startEnd(indexer::conveyorReverse, indexer::conveyorStop));
        // operator.x().whileTrue(
        //     Commands.startEnd(indexer::indexerForward, indexer::indexerStop));
        // operator.y().whileTrue(
        //     Commands.startEnd(indexer::indexerReverse, indexer::indexerStop));       

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