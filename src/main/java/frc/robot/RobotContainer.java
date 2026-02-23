// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
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

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shooter
        // =====================================================================

        // Right Trigger: Shoot with the currently selected preset.
        //
        // Default preset on startup: CLOSE (Close RPM + Close hood).
        // Cycle presets with POV Left / POV Right before shooting.
        //
        // Sequence: arm preset → ramp flywheel + move hood → wait until ready → feed
        // On release: stop indexer/conveyor → return shooter to standby.
        driver.rightTrigger(0.5).whileTrue(
            ShooterCommands.shootWithSelectedPreset(shooter, indexer)
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
        driver.leftTrigger(0.5).whileTrue(intake.intakeFuel());

        // Left Bumper: Retract slides immediately.
        driver.leftBumper().onTrue(Commands.runOnce(intake::retractSlides, intake));

      
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