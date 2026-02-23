package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

    public static final String PHASE1_RIGHT_CENTER_RUN  = "RightCenterRun";
    public static final String PHASE1_LEFT_CENTER_RUN   = "LeftCenterRun";
    public static final String PHASE2_RIGHT_TO_LEFT_RUN = "RightToLeftRun";
    public static final String PHASE2_DEPOT_RUN         = "DepotRun";
    public static final String PHASE2_LEFT_TO_RIGHT_RUN = "LeftToRightRun";

    // =========================================================================
    // INTAKE EVENT TIMESTAMPS (read directly from .traj files)
    // =========================================================================

    // RightCenterRun
    private static final double P1R_INTAKE_START = 0.0;
    private static final double P1R_INTAKE_STOP  = 3.698;

    // LeftCenterRun — update these once you have the .traj timestamps
    private static final double P1L_INTAKE_START = 0.0;
    private static final double P1L_INTAKE_STOP  = 3.698;

    // RightToLeftRun
    private static final double P2_INTAKE_START = 2.371;
    private static final double P2_INTAKE_STOP  = 6.446;

    // DepotRun
    private static final double P2D_INTAKE_START = 0.0;
    private static final double P2D_INTAKE_STOP  = 0.0;

    // LeftToRightRun — update these once you have the .traj timestamps
    private static final double P2LR_INTAKE_START = 0.0;
    private static final double P2LR_INTAKE_STOP  = 0.0;

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
    private final Optional<Trajectory<SwerveSample>> m_traj1Left =
        Choreo.loadTrajectory(PHASE1_LEFT_CENTER_RUN);
    private final Optional<Trajectory<SwerveSample>> m_traj2 =
        Choreo.loadTrajectory(PHASE2_RIGHT_TO_LEFT_RUN);
    private final Optional<Trajectory<SwerveSample>> m_traj2Depot =
        Choreo.loadTrajectory(PHASE2_DEPOT_RUN);
    private final Optional<Trajectory<SwerveSample>> m_traj2LeftRight =
        Choreo.loadTrajectory(PHASE2_LEFT_TO_RIGHT_RUN);

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
    // STARTING POSE HELPER
    // Returns the initial pose of the given path name, for seeding teleop odometry.
    // =========================================================================

    public Optional<Pose2d> getStartPose(String pathName) {
        return getTrajectory(pathName)
            .flatMap(traj -> traj.getInitialPose(false));
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
                    shootWindow(),
                    Commands.waitSeconds(2.0)
                ))
                .andThen(buildPathCommand(selected.get(i), false));
        }

        full = full
            .andThen(Commands.deadline(
                shootWindow(),
                Commands.waitSeconds(2.0)
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
                intakeStartTime = P1R_INTAKE_START;
                intakeStopTime  = P1R_INTAKE_STOP;
                break;
            case PHASE1_LEFT_CENTER_RUN:
                intakeStartTime = P1L_INTAKE_START;
                intakeStopTime  = P1L_INTAKE_STOP;
                break;
            case PHASE2_RIGHT_TO_LEFT_RUN:
                intakeStartTime = P2_INTAKE_START;
                intakeStopTime  = P2_INTAKE_STOP;
                break;
            case PHASE2_DEPOT_RUN:
                intakeStartTime = P2D_INTAKE_START;
                intakeStopTime  = P2D_INTAKE_STOP;
                break;
            case PHASE2_LEFT_TO_RIGHT_RUN:
                intakeStartTime = P2LR_INTAKE_START;
                intakeStopTime  = P2LR_INTAKE_STOP;
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
        PIDController pid = new PIDController(0.17, 0.00, 0.00);
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

            // Race: rotate + update RPM/hood vs feed-when-ready vs 2s timeout
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
                    Commands.waitUntil(m_shooter::isReady).withTimeout(2.0),
                    Commands.runOnce(() ->
                        System.out.println("[ShootWindow] Shooter ready — feeding now.")
                    ),
                    ShooterCommands.feedOnly(m_indexer)
                ),

                // Hard 2-second ceiling
                Commands.waitSeconds(2.0)
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
            case PHASE1_LEFT_CENTER_RUN:   return m_traj1Left;
            case PHASE2_RIGHT_TO_LEFT_RUN: return m_traj2;
            case PHASE2_DEPOT_RUN:         return m_traj2Depot;
            case PHASE2_LEFT_TO_RIGHT_RUN: return m_traj2LeftRight;
            default: return Optional.empty();
        }
    }
}