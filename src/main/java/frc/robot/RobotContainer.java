// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.generated.TunerConstants;
import frc.robot.commands.HubTrackingCommand;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.indexer.IndexerIOHardware;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
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
    private VisionSubsystem        vision;
    private final LedSubsystem     ledSubsystem;
    // private final ClimberSubsystem climber;

    // Kept as a field so Robot.java can call updateSimVision() every sim tick.
    // Null when running on real hardware.
    private final VisionIOSim visionIOSim;

    // Keep a direct reference to the Limelight IO so we can call setIMUMode()
    // from triggers. Null when running in simulation.
    private final VisionIOLimelight visionIOLimelight;

    // ===== Auto =====
    private final AutoFactory  autoFactory;
    private final AutoRoutines autoRoutines;

    private final SendableChooser<String> phase1Chooser = new SendableChooser<>();
    private final SendableChooser<String> phase2Chooser = new SendableChooser<>();
    private final SendableChooser<String> phase3Chooser = new SendableChooser<>();

    // =========================================================================
    // ALL AUTO PATHS — add new paths here and they'll appear in all 3 choosers
    // =========================================================================
    private static final String[][] ALL_AUTO_PATHS = {
        // { "Display Name",                    AutoRoutines.CONSTANT                  }
        { "Right Center Run",                AutoRoutines.PHASE1_RIGHT_CENTER_RUN   },
        { "Left Center Run",                 AutoRoutines.PHASE1_LEFT_CENTER_RUN    },
        { "Right To Left Run",               AutoRoutines.PHASE2_RIGHT_TO_LEFT_RUN  },
        { "Depot Run (Phase 2)",             AutoRoutines.PHASE2_DEPOT_RUN          },
        { "Left To Right Run",               AutoRoutines.PHASE2_LEFT_TO_RIGHT_RUN  },
    };

    public RobotContainer() {
        intake  = new IntakeSubsystem(new IntakeIOHardware());
        indexer = new IndexerSubsystem(new IndexerIOHardware());
        shooter = new ShooterSubsystem(new ShooterIOHardware());

        // Automatically swap between sim and real Limelight.
        if (RobotBase.isSimulation()) {
            visionIOSim       = new VisionIOSim();
            visionIOLimelight = null;
        } else {
            visionIOSim       = null;
            visionIOLimelight = new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME);
        }

        VisionIO visionIO = (visionIOSim != null)
                ? visionIOSim
                : visionIOLimelight;

        // Temporary holder so the reset lambda can reference vision before assignment completes
        VisionSubsystem[] visionHolder = new VisionSubsystem[1];

        visionHolder[0] = new VisionSubsystem(
            visionIO,
            drivetrain::addVisionMeasurement,
            () -> drivetrain.resetPoseToVision(visionHolder[0].getLastAcceptedPose()),
            () -> drivetrain.getState().Pose,
            () -> drivetrain.getState().Pose.getRotation().getDegrees()
        );

        vision = visionHolder[0];

        ledSubsystem = new LedSubsystem();
        // climber = new ClimberSubsystem();

        autoFactory  = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, intake, shooter, indexer, vision);

        // ===== Auto Choosers =====
        phase1Chooser.setDefaultOption(AutoRoutines.NONE, AutoRoutines.NONE);
        phase2Chooser.setDefaultOption(AutoRoutines.NONE, AutoRoutines.NONE);
        phase3Chooser.setDefaultOption(AutoRoutines.NONE, AutoRoutines.NONE);

        for (String[] path : ALL_AUTO_PATHS) {
            phase1Chooser.addOption(path[0], path[1]);
            phase2Chooser.addOption(path[0], path[1]);
            phase3Chooser.addOption(path[0], path[1]);
        }

        SmartDashboard.putData("Auto Phase 1", phase1Chooser);
        SmartDashboard.putData("Auto Phase 2", phase2Chooser);
        SmartDashboard.putData("Auto Phase 3", phase3Chooser);

        configureBindings();
    }

    // -------------------------------------------------------------------------
    // Sim update — call from Robot.simulationPeriodic()
    // -------------------------------------------------------------------------

    public void updateSimVision() {
        if (visionIOSim != null) {
            visionIOSim.setSimRobotPose(drivetrain.getState().Pose);
        }
    }

    // -------------------------------------------------------------------------
    // Bindings
    // -------------------------------------------------------------------------

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

        // =====================================================================
        // LL4 IMU MODE SWITCHING
        // While disabled: mode 1 (EXTERNAL_SEED) — continuously seeds the internal
        // IMU with the robot's gyro so it knows its orientation before the match.
        // When enabled: mode 4 (INTERNAL_EXTERNAL_ASSIST) — internal IMU runs at
        // 1kHz for smooth per-frame updates; external gyro corrects drift over time.
        // =====================================================================
        if (visionIOLimelight != null) {
            RobotModeTriggers.disabled().whileTrue(
                Commands.run(() -> visionIOLimelight.setIMUMode(1)).ignoringDisable(true)
            );

            new Trigger(() -> DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled())
                .onTrue(Commands.runOnce(() -> visionIOLimelight.setIMUMode(4)));
        }

        // Seed drivetrain pose from selected Phase 1 auto start position when teleop begins.
        new Trigger(() -> DriverStation.isTeleopEnabled()).onTrue(
            drivetrain.runOnce(() -> {
                String selected = phase1Chooser.getSelected();
                if (selected != null && !selected.equals(AutoRoutines.NONE)) {
                    autoRoutines.getStartPose(selected).ifPresent(pose -> {
                        drivetrain.resetPose(pose);
                        System.out.println("[RobotContainer] Teleop pose seeded from auto: " + selected + " -> " + pose);
                    });
                }
            })
        );

        // Start: Reset field-centric heading
        driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shooter Presets
        // =====================================================================

        // A: Arm close shot
        driver.a().onTrue(ShooterCommands.armCloseShot(shooter));

        // B: Arm pass shot
        driver.b().onTrue(ShooterCommands.armPassShot(shooter));

        // X: Arm hub shot and begin spinning up
        driver.x().onTrue(new InstantCommand(shooter::hubShot, shooter));

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Shoot
        // =====================================================================

        Trigger hubShotArmed = new Trigger(shooter::isHubShotArmed);

        // RT (hub shot armed): Rotate toward hub, update RPM/hood from distance,
        // and feed automatically once shooter is ready.
        driver.rightTrigger(0.5).and(hubShotArmed).whileTrue(
            new HubTrackingCommand(
                drivetrain, shooter, indexer, vision,
                () -> -driver.getLeftY() * MaxSpeed,
                () -> -driver.getLeftX() * MaxSpeed
            )
        );

        // RT (standard shot): Close or pass shot at currently armed preset.
        driver.rightTrigger(0.5).and(hubShotArmed.negate()).whileTrue(
            ShooterCommands.shootAtCurrentTarget(shooter, indexer)
        );

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Intake
        // =====================================================================

        // Left Trigger: Extend slides, run roller, and slowly spin shooter/indexer
        // at IDLE_RPM while held. On release, shooter returns to SPINUP and the
        // previously armed preset is preserved.
        driver.leftTrigger(0.5).whileTrue(
            intake.intakeFuel()
                .alongWith(shooter.intakeModeCommand())
        );

        // Left Bumper: Retract intake and slowly reverse roller while held.
        driver.leftBumper().whileTrue(intake.retractAndReverseRollerCommand());

        // Right Bumper: Run indexer and conveyor to pull game pieces through the hopper.
        driver.rightBumper().whileTrue(indexer.intakeCommand());

        // =====================================================================
        // DRIVER CONTROLLER (Port 0) - Commented out (TODO: enable as needed)
        // =====================================================================

        // driver.povLeft()  — free to reassign
        // driver.rightBumper()...
        // driver.y().whileTrue(Commands.startEnd(indexer::indexerForward, indexer::indexerStop));
        // driver.povUp().onTrue(ShooterCommands.increaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // driver.povDown().onTrue(ShooterCommands.decreaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // driver.y().whileTrue(intake.ejectFuel());

        // =====================================================================
        // OPERATOR CONTROLLER (Port 1) - Commented out (TODO: enable as needed)
        // =====================================================================

        // operator.povUp().onTrue(ShooterCommands.increaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // operator.povDown().onTrue(ShooterCommands.decreaseTargetHoodPose(shooter, ShooterSubsystem.HOOD_TEST_INCREMENT));
        // operator.a().whileTrue(Commands.startEnd(indexer::conveyorForward, indexer::conveyorStop));
        // operator.b().whileTrue(Commands.startEnd(indexer::conveyorReverse, indexer::conveyorStop));
        // operator.x().whileTrue(Commands.startEnd(indexer::indexerForward, indexer::indexerStop));
        // operator.y().whileTrue(Commands.startEnd(indexer::indexerReverse, indexer::indexerStop));
        // operator.start().onTrue(climber.stopClimber());

        // =====================================================================
        // OPERATOR PERSPECTIVE
        // =====================================================================
        drivetrain.setOperatorPerspectiveForward(
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-90)
        );
    }

    public Command getAutonomousCommand() {
        return autoRoutines.buildFullAuto(
            phase1Chooser.getSelected(),
            phase2Chooser.getSelected(),
            phase3Chooser.getSelected()
        );
    }

    public void updateGameData() {
        gameDataTelemetry.update();
    }

    public GameDataTelemetry getGameDataTelemetry() {
        return gameDataTelemetry;
    }
}