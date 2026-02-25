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
        // Right Trigger: Vision-aligned shot.
        //
        // While held:
        //   - Starts spinning at the current POV-selected preset immediately.
        //   - If a hub AprilTag is visible: continuously updates RPM + hood from
        //     the distance lookup table and drives rotation to center the tag.
        //   - If no hub tag is visible: holds current preset, no rotation override.
        //   - Fires indexer/conveyor as soon as vision is aligned AND shooter is ready.
        //   - Driver left stick still controls translation freely throughout.
        //
        // Cycle the fallback preset with POV Left / POV Right before or during hold.
        // Active preset name is published to Shooter/SelectedPreset on NetworkTables.
        driver.rightTrigger(0.5).and(driver.a()).whileTrue(
            FuelCommands.visionAlignAndShoot(
                shooter, vision, indexer, drivetrain,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed
            )
        );

        // POV Right: Cycle to next preset (Close → Tower → Trench → Pass → Far → Close).
        // POV Left:  Cycle to previous preset (Close → Far → Pass → Trench → Tower → Close).
        // Selected preset is published to Shooter/SelectedPreset on NetworkTables / Elastic.
        // No subsystem requirement — safe to press while trigger is held without interrupting a shot.
        driver.povRight().onTrue(Commands.runOnce(shooter::cyclePresetForward));
        driver.povLeft().onTrue(Commands.runOnce(shooter::cyclePresetBackward));

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