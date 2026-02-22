package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.AutoAlignAndShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoRoutines {

    // =========================================================================
    // PHASE OPTION CONSTANTS
    // Use these in RobotContainer's SendableChoosers so names stay in sync.
    // =========================================================================

    // Phase 1 options
    public static final String PHASE1_CENTER_RUN  = "BR_CenterRun";
    public static final String PHASE1_CHUTE_CATCH = "BR_ChuteCatch"; // TODO: add trajectory file

    // Phase 2 options
    public static final String PHASE2_SWITCH_RUN = "BR_2_SwitchRun";
    public static final String PHASE2_CLIMB      = "BR_2_Climb";

    // Phase 3 options — expand as needed
    public static final String PHASE3_NONE = "None";

    // =========================================================================
    // SUBSYSTEM REFERENCES
    // =========================================================================

    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;
    private final IndexerSubsystem m_indexer;
    private final VisionSubsystem m_vision;

    public AutoRoutines(
            AutoFactory factory,
            CommandSwerveDrivetrain drivetrain,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            VisionSubsystem vision) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_shooter = shooter;
        m_indexer = indexer;
        m_vision = vision;
    }

    // =========================================================================
    // MULTI-PHASE AUTO BUILDER
    // Called by RobotContainer.getAutonomousCommand() with the three chooser
    // selections. Returns a single Command that runs all phases sequentially.
    // =========================================================================

    /**
     * Builds a complete auto command by chaining the selected phase commands.
     *
     * @param phase1 Phase 1 selection string (use PHASE1_* constants)
     * @param phase2 Phase 2 selection string (use PHASE2_* constants)
     * @param phase3 Phase 3 selection string (use PHASE3_* constants)
     * @return A sequential command running all three phases
     */
    public Command buildAuto(String phase1, String phase2, String phase3) {
        Command p1 = buildPhase1(phase1);
        Command p2 = buildPhase2(phase2);
        Command p3 = buildPhase3(phase3);

        // Chain all three phases sequentially.
        // If a phase returns Commands.none(), it is skipped cleanly.
        return p1.andThen(p2).andThen(p3);
    }

    // =========================================================================
    // PHASE BUILDERS
    // Each method maps a selection string to the appropriate routine.
    // Add new cases here as you build out more autos.
    // =========================================================================

    private Command buildPhase1(String selection) {
        switch (selection) {
            case PHASE1_CENTER_RUN:
                return phase1_CenterRun();
            case PHASE1_CHUTE_CATCH:
                return phase1_ChuteCatch();
            default:
                return Commands.none();
        }
    }

    private Command buildPhase2(String selection) {
        switch (selection) {
            case PHASE2_SWITCH_RUN:
                return phase2_SwitchRun();
            case PHASE2_CLIMB:
                return phase2_Climb();
            default:
                return Commands.none();
        }
    }

    private Command buildPhase3(String selection) {
        // Add phase 3 cases here as you build them out
        switch (selection) {
            default:
                return Commands.none();
        }
    }

    // =========================================================================
    // PHASE 1 ROUTINES
    // =========================================================================

    /**
     * Phase 1: Drive the center run path, intake fuel, then align and shoot.
     */
    private Command phase1_CenterRun() {
        final AutoRoutine routine = m_factory.newRoutine("BR_CenterRun");
        final AutoTrajectory path = routine.trajectory("BR_CenterRun");

        routine.active().onTrue(
            path.resetOdometry()
                .andThen(path.cmd().alongWith(
                    m_shooter.runOnce(m_shooter::closeShot)
                ))
                .andThen(new AutoAlignAndShootCommand(
                    m_drivetrain, m_shooter, m_vision, m_indexer, 5.0
                ))
        );

        path.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());

        return routine.cmd();
    }

    /**
     * Phase 1: Drive the chute catch path, intake fuel, then align and shoot.
     * TODO: Add BR_ChuteCatch.traj and event markers, then implement this method.
     */
    private Command phase1_ChuteCatch() {
        // TODO: implement once BR_ChuteCatch.traj is ready
        return Commands.none();
    }

    // =========================================================================
    // PHASE 2 ROUTINES
    // =========================================================================

    /**
     * Phase 2: Drive the switch run path and intake fuel.
     */
    private Command phase2_SwitchRun() {
        final AutoRoutine routine = m_factory.newRoutine("BR_2_SwitchRun");
        final AutoTrajectory path = routine.trajectory("BR_2_SwitchRun");

        routine.active().onTrue(path.cmd());

        path.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());

        return routine.cmd();
    }

    /**
     * Phase 2: Drive to the climb position and operate the climber.
     */
    private Command phase2_Climb() {
        final AutoRoutine routine = m_factory.newRoutine("BR_2_Climb");
        final AutoTrajectory path = routine.trajectory("BR_2_Climb");

        routine.active().onTrue(path.cmd());

        // TODO: bind to climber subsystem once implemented
        path.atTime("RetractIntake").onTrue(m_intake.stopFuel());
        // path.atTime("PauseExtendClimber").onTrue(climber.pauseExtend());
        // path.atTime("RetractClimber").onTrue(climber.retract());

        return routine.cmd();
    }
}