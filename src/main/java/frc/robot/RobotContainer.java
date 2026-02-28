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
<<<<<<< HEAD
import frc.robot.commands.AutoHubShootCommand;
=======
>>>>>>> parent of d00b6cc (Added all vision code back into system, tables talking with LL4 OK)
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
    private final LedSubsystem     larson;
    private final FuelCommands     fuelCommands = null;
    private final AutoFactory      autoFactory;
    private final AutoRoutines     autoRoutines;
    private final AutoChooser      autoChooser = new AutoChooser();

    public RobotContainer() {
        intake  = new IntakeSubsystem(new IntakeIOHardware());
        indexer = new IndexerSubsystem(new IndexerIOHardware());
        shooter = new ShooterSubsystem(new ShooterIOHardware());

        // VisionSubsystem requires:
        //   1. A VisionIO implementation
        //   2. A pose consumer callback  (drivetrain::addVisionMeasurement)
        //   3. A reset-pose callback     (called once on first valid tag)
        //   4. A pose supplier           (current robot pose from odometry)
        //   5. A yaw supplier            (current robot heading in degrees)
        // The reset callback cannot reference `vision` directly in the lambda
        // because `vision` is final and not yet assigned at construction time.
        // We use a single-element array as a mutable holder so the lambda can
        // close over the holder and reach the VisionSubsystem once it exists.
        VisionSubsystem[] visionHolder = new VisionSubsystem[1];
        visionHolder[0] = new VisionSubsystem(
            new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME),
            drivetrain::addVisionMeasurement,
            () -> drivetrain.resetPose(visionHolder[0].getLastAcceptedPose()),
            () -> drivetrain.getState().Pose,
            () -> drivetrain.getState().Pose.getRotation().getDegrees()
        );
        vision = visionHolder[0];

        larson = new LedSubsystem();

        autoFactory  = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, indexer, intake, shooter, fuelCommands);

        SmartDashboard.putData(autoChooser);
        autoChooser.addRoutine("FM",                    autoRoutines::FM);
        autoChooser.addRoutine("B",                     autoRoutines::B);
        autoChooser.addRoutine("Lob",                   autoRoutines::Lob);
        autoChooser.addRoutine("Default(Run this one)", autoRoutines::Default);
        autoChooser.addRoutine("The Big D",             autoRoutines::Dummy);
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
            FuelCommands.shootWithSelectedPreset(shooter, indexer)
        );

<<<<<<< HEAD
        // X Button: Full auto hub shot — snaps to hub, interpolates RPM + hood from distance,
        // feeds only when aligned AND shooter is ready.
        driver.x().whileTrue(new AutoHubShootCommand(
            drivetrain,
            vision,
            shooter,
            indexer,
            () -> -driver.getLeftY() * MaxSpeed,
            () -> -driver.getLeftX() * MaxSpeed
        ));

        // POV Right: Cycle to next preset (Close → Tower → Trench → Pass → Far → Close).
        // POV Left:  Cycle to previous preset.
        driver.povRight().onTrue(Commands.runOnce(shooter::cyclePresetForward));
        driver.povLeft().onTrue(Commands.runOnce(shooter::cyclePresetBackward));
=======
        // Face buttons select and latch preset (selection sticks after button released).
        // A = CLOSE, B = TRENCH, X = TOWER, Y = FAR
        // Right Bumper = PASS
        operator.a().onTrue(Commands.runOnce(() -> shooter.selectPreset(ShooterSubsystem.ShotPreset.CLOSE)));
        operator.b().onTrue(Commands.runOnce(() -> shooter.selectPreset(ShooterSubsystem.ShotPreset.TRENCH)));
        operator.x().onTrue(Commands.runOnce(() -> shooter.selectPreset(ShooterSubsystem.ShotPreset.TOWER)));
        operator.y().onTrue(Commands.runOnce(() -> shooter.selectPreset(ShooterSubsystem.ShotPreset.FAR)));
        operator.rightBumper().onTrue(Commands.runOnce(() -> shooter.selectPreset(ShooterSubsystem.ShotPreset.PASS)));
        // TODO: Test the air popper command and tune the popper RPM and hood pose. Consider adding to intakeFuel()
        operator.a().whileTrue(FuelCommands.runAirPopper(indexer, shooter)); 

        // TODO: Test the air popper command while running the intake.
        // operator.b().whileTrue(FuelCommands.runAirPopper(indexer, shooter).alongWith(intake.intakeFuel())); 
>>>>>>> parent of d00b6cc (Added all vision code back into system, tables talking with LL4 OK)

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Intake
        // =====================================================================

        // Left Trigger: Extend slides and run roller while held.
        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // Left Bumper: Retract slides incrementally.
        driver.leftBumper().whileTrue(intake.compressFuelIncremental());
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