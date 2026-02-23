package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoRoutines {

    // =========================================================================
    // PHASE OPTION CONSTANTS
    // =========================================================================

    public static final String NONE = "[NONE]";

    // Phase 1
    public static final String PHASE1_RIGHT_CENTER_RUN = "Blue1_RightCenterRun";

    // Phase 2
    public static final String PHASE2_RIGHT_TO_LEFT_RUN = "Blue2_RightToLeftRun";

    // Phase 3
    public static final String PHASE3_DEPOT_RUN = "Blue3_DepotRun";

    // =========================================================================
    // SUBSYSTEM REFERENCES
    // =========================================================================

    private final AutoFactory             m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final IntakeSubsystem         m_intake;
    private final ShooterSubsystem        m_shooter;
    private final IndexerSubsystem        m_indexer;
    private final VisionSubsystem         m_vision;

    public AutoRoutines(
            AutoFactory factory,
            CommandSwerveDrivetrain drivetrain,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            VisionSubsystem vision) {
        m_factory    = factory;
        m_drivetrain = drivetrain;
        m_intake     = intake;
        m_shooter    = shooter;
        m_indexer    = indexer;
        m_vision     = vision;
    }

    // =========================================================================
    // PHASE BUILDERS — called by RobotContainer
    // =========================================================================

    public Command buildPhase1(String selection) {
        switch (selection) {
            case PHASE1_RIGHT_CENTER_RUN: return phase1_RightCenterRun();
            default: return Commands.none();
        }
    }

    public Command buildPhase2(String selection) {
        switch (selection) {
            case PHASE2_RIGHT_TO_LEFT_RUN: return phase2_RightToLeftRun();
            default: return Commands.none();
        }
    }

    public Command buildPhase3(String selection) {
        switch (selection) {
            case PHASE3_DEPOT_RUN: return phase3_DepotRun();
            default: return Commands.none();
        }
    }

    // =========================================================================
    // PHASE 1 ROUTINES
    // =========================================================================

    private Command phase1_RightCenterRun() {
        final AutoRoutine routine = m_factory.newRoutine("Blue1_RightCenterRun");
        final AutoTrajectory path = routine.trajectory("Blue1_RightCenterRun");

        routine.active().onTrue(
            path.resetOdometry()
                .andThen(path.cmd())
        );

        // RunIntake fires at the very start (waypoint 0)
        path.atTime("RunIntake").onTrue(m_intake.intakeFuel());

        // StopIntake fires at waypoint 4 (t ≈ 3.70s) when returning home
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());

        // StartShooting fires at the end — arm shooter and fire
        path.atTime("StartShooting").onTrue(
            ShooterCommands.shootSequence(
                m_shooter, m_indexer,
                ShooterSubsystem.CLOSE_SHOT_RPM,
                ShooterSubsystem.CLOSE_SHOT_HOOD
            )
        );

        return routine.cmd();
    }

    // =========================================================================
    // PHASE 2 ROUTINES
    // =========================================================================

    private Command phase2_RightToLeftRun() {
        final AutoRoutine routine = m_factory.newRoutine("Blue2_RightToLeftRun");
        final AutoTrajectory path = routine.trajectory("Blue2_RightToLeftRun");

        routine.active().onTrue(path.cmd());

        // StartIntake fires at waypoint 3 (t ≈ 2.37s) when heading toward the left side
        path.atTime("StartIntake").onTrue(m_intake.intakeFuel());

        // StopIntake fires at waypoint 6 (t ≈ 6.45s) before returning to shoot position
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());

        // StartShooting fires at the end at waypoint 7
        path.atTime("StartShooting").onTrue(
            ShooterCommands.shootSequence(
                m_shooter, m_indexer,
                ShooterSubsystem.CLOSE_SHOT_RPM,
                ShooterSubsystem.CLOSE_SHOT_HOOD
            )
        );

        return routine.cmd();
    }

    // =========================================================================
    // PHASE 3 ROUTINES
    // =========================================================================

    private Command phase3_DepotRun() {
        final AutoRoutine routine = m_factory.newRoutine("Blue3_DepotRun");
        final AutoTrajectory path = routine.trajectory("Blue3_DepotRun");

        routine.active().onTrue(path.cmd());

        // StartIntake fires at the very start (waypoint 0) — collecting from depot
        path.atTime("StartIntake").onTrue(m_intake.intakeFuel());

        // StopIntake and StartShooting both fire at the end (waypoint 3, return home)
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());
        path.atTime("StartShooting").onTrue(
            ShooterCommands.shootSequence(
                m_shooter, m_indexer,
                ShooterSubsystem.CLOSE_SHOT_RPM,
                ShooterSubsystem.CLOSE_SHOT_HOOD
            )
        );

        return routine.cmd();
    }
}