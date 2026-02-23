package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutoRoutines {

    // =========================================================================
    // PHASE OPTION CONSTANTS
    // =========================================================================

    public static final String NONE = "[NONE]";

    public static final String PHASE1_RIGHT_CENTER_RUN  = "Blue1_RightCenterRun";
    public static final String PHASE2_RIGHT_TO_LEFT_RUN = "Blue2_RightToLeftRun";
    public static final String PHASE3_DEPOT_RUN         = "Blue3_DepotRun";

    // =========================================================================
    // INTAKE EVENT TIMESTAMPS (read directly from .traj files)
    // =========================================================================

    // Blue1_RightCenterRun
    private static final double P1_INTAKE_START = 0.0;
    private static final double P1_INTAKE_STOP  = 3.698;

    // Blue2_RightToLeftRun
    private static final double P2_INTAKE_START = 2.371;
    private static final double P2_INTAKE_STOP  = 6.446;

    // Blue3_DepotRun
    private static final double P3_INTAKE_START = 0.0;
    private static final double P3_INTAKE_STOP  = 4.846;

    // =========================================================================
    // SUBSYSTEM REFERENCES
    // =========================================================================

    private final CommandSwerveDrivetrain m_drivetrain;
    private final IntakeSubsystem         m_intake;
    private final ShooterSubsystem        m_shooter;
    private final IndexerSubsystem        m_indexer;
    private final VisionSubsystem         m_vision;

    // Load all trajectories once at startup — never load inside a command
    private final Optional<Trajectory<SwerveSample>> m_traj1 =
        Choreo.loadTrajectory(PHASE1_RIGHT_CENTER_RUN);
    private final Optional<Trajectory<SwerveSample>> m_traj2 =
        Choreo.loadTrajectory(PHASE2_RIGHT_TO_LEFT_RUN);
    private final Optional<Trajectory<SwerveSample>> m_traj3 =
        Choreo.loadTrajectory(PHASE3_DEPOT_RUN);

    public AutoRoutines(
            AutoFactory factory,
            CommandSwerveDrivetrain drivetrain,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            VisionSubsystem vision) {
        m_drivetrain = drivetrain;
        m_intake     = intake;
        m_shooter    = shooter;
        m_indexer    = indexer;
        m_vision     = vision;
    }

    // =========================================================================
    // MAIN BUILDER
    // =========================================================================

    public Command buildFullAuto(String p1, String p2, String p3) {
    List<String> selected = new ArrayList<>();
    if (!p1.equals(NONE)) selected.add(p1);
    if (!p2.equals(NONE)) selected.add(p2);
    if (!p3.equals(NONE)) selected.add(p3);

    if (selected.isEmpty()) return Commands.none();

    Command full = buildPathCommand(selected.get(0), true);

    for (int i = 1; i < selected.size(); i++) {
        full = full
            .andThen(Commands.deadline(
                Commands.waitSeconds(2.0), // 2s timer drives the duration
                shootWindow()              // shooter runs alongside, gets cut off at 2s
            ))
            .andThen(buildPathCommand(selected.get(i), false));
    }

    full = full
        .andThen(Commands.deadline(
            Commands.waitSeconds(2.0),
            shootWindow()
        ))
        .andThen(Commands.runOnce(m_shooter::returnToIdle, m_shooter));

    return full;
}

    // =========================================================================
    // PATH COMMAND BUILDER
    // =========================================================================

    private Command buildPathCommand(String name, boolean resetOdometry) {
        Optional<Trajectory<SwerveSample>> trajOpt = getTrajectory(name);

        if (trajOpt.isEmpty()) {
            System.err.println("[AutoRoutines] WARNING: trajectory not found: " + name);
            return Commands.none();
        }

        Trajectory<SwerveSample> trajectory = trajOpt.get();
        double totalTime = trajectory.getTotalTime();
        Timer timer = new Timer();

        double intakeStartTime;
        double intakeStopTime;

        switch (name) {
            case PHASE1_RIGHT_CENTER_RUN:
                intakeStartTime = P1_INTAKE_START;
                intakeStopTime  = P1_INTAKE_STOP;
                break;
            case PHASE2_RIGHT_TO_LEFT_RUN:
                intakeStartTime = P2_INTAKE_START;
                intakeStopTime  = P2_INTAKE_STOP;
                break;
            case PHASE3_DEPOT_RUN:
                intakeStartTime = P3_INTAKE_START;
                intakeStopTime  = P3_INTAKE_STOP;
                break;
            default:
                intakeStartTime = -1;
                intakeStopTime  = -1;
        }

        return new FunctionalCommand(
            // onInit
            () -> {
                System.out.println("[AutoRoutines] Starting path: " + name);
                if (resetOdometry) {
                    trajectory.getInitialPose(false).ifPresent(m_drivetrain::resetPose);
                }
                timer.reset();
                timer.start();
            },

            // onExecute
            () -> {
                double t = timer.get();

                trajectory.sampleAt(t, false).ifPresent(m_drivetrain::followPath);

                if (intakeStartTime >= 0 && Math.abs(t - intakeStartTime) < 0.02) {
                    m_intake.intakeFuel().schedule();
                }
                if (intakeStopTime >= 0 && Math.abs(t - intakeStopTime) < 0.02) {
                    m_intake.stopFuel().schedule();
                }
            },

          // onEnd
        interrupted -> {
            System.out.println("[AutoRoutines] Path ended: " + name + " | interrupted=" + interrupted);
            timer.stop();
            m_drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake());
        },
            // isFinished
            () -> timer.hasElapsed(totalTime),

            m_drivetrain
        ).withName("FollowTrajectory_" + name);
    }

    // =========================================================================
    // SHOOT WINDOW
    // =========================================================================

    private Command shootWindow() {
        PIDController pid = new PIDController(0.10, 0.00, 0.00);
        pid.setSetpoint(0.0);
        pid.setTolerance(1.0);
        pid.enableContinuousInput(-180.0, 180.0);

        return Commands.sequence(

            // Wait for vision to acquire the hub before doing anything else
            Commands.waitUntil(() -> {
                boolean acquired = m_vision.getDistanceToHub() > 0;
                if (!acquired) System.out.println("[ShootWindow] Waiting for vision to acquire hub...");
                return acquired;
            }).withTimeout(1.0),

            // Spin up shooter and set hub shot preset
            Commands.runOnce(() -> {
                System.out.println("[ShootWindow] Vision acquired. Spinning up shooter.");
                m_shooter.setHubShotPreset();
                m_shooter.prepareToShoot();
                pid.reset();
            }, m_shooter),

            // Race: rotate + update RPM/hood vs feed-when-ready vs 4s timeout
            new ParallelRaceGroup(
                // Rotate to face hub and live-update RPM/hood from distance
                Commands.run(() -> {
    double dist = m_vision.getDistanceToHub();
    double angle = m_vision.getAngleToHub();

    if (dist > 0) m_shooter.updateFromDistance(dist);

    double rot = pid.calculate(angle);
    rot = Math.max(-3.0, Math.min(3.0, rot));

    m_drivetrain.setControl(
        new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rot)
        );
        }, m_drivetrain),

                // Wait until ready then feed — ends race group when feedOnly finishes
                Commands.sequence(
                    Commands.waitUntil(m_shooter::isReady).withTimeout(3.0),
                    Commands.runOnce(() ->
                        System.out.println("[ShootWindow] Shooter ready — feeding now.")
                    ),
                    ShooterCommands.feedOnly(m_indexer)
                ),

                // Hard 4-second ceiling
                Commands.waitSeconds(4.0)
            ),

            // Clean up after the window closes
            Commands.runOnce(() -> {
                System.out.println("[ShootWindow] Window closed. Cleaning up.");
                m_indexer.indexerStop();
                m_indexer.conveyorStop();
                m_shooter.returnToIdle();
                m_drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.Idle());
            }, m_drivetrain, m_shooter, m_indexer)

        ).withName("ShootWindow");
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    private Optional<Trajectory<SwerveSample>> getTrajectory(String name) {
        switch (name) {
            case PHASE1_RIGHT_CENTER_RUN:  return m_traj1;
            case PHASE2_RIGHT_TO_LEFT_RUN: return m_traj2;
            case PHASE3_DEPOT_RUN:         return m_traj3;
            default: return Optional.empty();
        }
    }
}