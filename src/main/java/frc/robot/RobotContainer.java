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
import frc.robot.commands.SnapToHubCommand;
import frc.robot.commands.AutoHubShootCommand;
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
        autoChooser.addRoutine("FM", autoRoutines::FM);
        autoChooser.addRoutine("B", autoRoutines::B);
        autoChooser.addRoutine("Lob", autoRoutines::Lob);
        autoChooser.addRoutine("Default(Run this one)", autoRoutines::Default);
        autoChooser.addRoutine("The Big D", autoRoutines::Dummy);
        // ✅ Publish to NetworkTables so Elastic can display it as a ComboBox Chooser  
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

        driver.povDown().whileTrue(
            new SnapToHubCommand(
                drivetrain,
                vision,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Start: Reset field-centric heading
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // ===== MegaTag2 Vision Pose Estimation =====
        // The telemetry callback runs every odometry loop (~250Hz on CANivore),
        // so this is the right place to both push orientation and consume the pose.
        drivetrain.registerTelemetry(state -> {
            // 1. Give the Limelight the current gyro yaw so MegaTag2 can use it
            vision.setRobotOrientation(state.Pose.getRotation().getDegrees());

            // 2. If we have a valid estimate, feed it to the pose estimator
            if (vision.hasMegaTag2Estimate()) {
                double dist = vision.getMegaTag2AvgTagDist();

                // Trust scales quadratically with distance.
                // At 1m: xyStd = 0.5. At 3m: xyStd = 4.5.
                // Increase the multiplier (0.5) if vision is jerking the pose around.
                double xyStdDev = 0.5 * dist * dist;

                // Don't correct rotation — MegaTag2 derives it from YOUR gyro,
                // so feeding it back would create a feedback loop.
                double rotStdDev = 9999999.0;

                drivetrain.addVisionMeasurement(
                    vision.getMegaTag2Pose(),
                    vision.getMegaTag2Timestamp(),
                    edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
                );
            }
        });

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shooter
        // =====================================================================

        driver.rightTrigger(0.5).whileTrue(
            FuelCommands.shootWithSelectedPreset(shooter, indexer)
        );
        // Right Trigger + Vision: Commented out — vision shot disabled for now.
        // driver.rightTrigger(0.5).and(driver.a()).whileTrue(
        //     FuelCommands.visionAlignAndShoot(
        //         shooter, vision, indexer, drivetrain,
        //         () -> -driver.getLeftY() * MaxSpeed,
        //         () -> -driver.getLeftX() * MaxSpeed
        //     )
        // );

<<<<<<< Updated upstream
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
=======
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
        // POV Left:  Cycle to previous preset (Close → Far → Pass → Trench → Tower → Close).

        // POV Right: Cycle to next preset (Close → Tower → Trench → Pass → Far → Close).
        // POV Left:  Cycle to previous preset (Close → Far → Pass → Trench → Tower → Close).
        // Selected preset is published to Shooter/SelectedPreset on NetworkTables / Elastic.
        // No subsystem requirement — safe to press while trigger is held without interrupting a shot.
        driver.povRight().onTrue(Commands.runOnce(shooter::cyclePresetForward));
        driver.povLeft().onTrue(Commands.runOnce(shooter::cyclePresetBackward));
>>>>>>> Stashed changes

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Intake
        // =====================================================================

        // Left Trigger: Extend slides and run roller while held.
        // TODO Check if this is proper way to call the command `intakeFuel()` from IntakeSubsytem
        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // Left Bumper: Retract slides immediately.
        driver.leftBumper().whileTrue(intake.compressFuelIncremental()); // TODO: Test

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