package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoRoutines {

    // =========================================================================
    // PHASE OPTION CONSTANTS
    // Add a constant here for each new routine you create, then register it
    // in RobotContainer's buildPhase1/2/3 switch statements.
    // =========================================================================

    public static final String NONE = "[NONE]";

    // Phase 1 options — add yours here:
    // public static final String PHASE1_MY_ROUTINE = "MyRoutineName";

    // Phase 2 options — add yours here:
    // public static final String PHASE2_MY_ROUTINE = "MyRoutineName";

    // Phase 3 options — add yours here:
    // public static final String PHASE3_MY_ROUTINE = "MyRoutineName";

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
            // Add cases here as you create routines, e.g.:
            // case PHASE1_MY_ROUTINE: return myRoutine();
            default: return Commands.none();
        }
    }

    public Command buildPhase2(String selection) {
        switch (selection) {
            // Add cases here as you create routines, e.g.:
            // case PHASE2_MY_ROUTINE: return myRoutine();
            default: return Commands.none();
        }
    }

    public Command buildPhase3(String selection) {
        switch (selection) {
            // Add cases here as you create routines, e.g.:
            // case PHASE3_MY_ROUTINE: return myRoutine();
            default: return Commands.none();
        }
    }

    // =========================================================================
    // ROUTINE IMPLEMENTATIONS — add your routines below
    // =========================================================================

    // Example structure for a routine:
    //
    // private Command myRoutine() {
    //     final AutoRoutine routine = m_factory.newRoutine("MyRoutineName");
    //     final AutoTrajectory path = routine.trajectory("MyRoutineName");
    //
    //     routine.active().onTrue(
    //         path.resetOdometry().andThen(path.cmd())
    //     );
    //
    //     path.atTime("SomeEvent").onTrue(Commands.runOnce(() -> {}));
    //
    //     return routine.cmd();
    // }
}