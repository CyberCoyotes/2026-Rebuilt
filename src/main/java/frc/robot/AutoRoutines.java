package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.AlignToHubCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * AutoRoutines — Autonomous routine definitions.
 */
public class AutoRoutines {

    // =========================================================================
    // PHASE OPTION CONSTANTS
    // =========================================================================

    public static final String PHASE1_CENTER_RUN  = "BR_CenterRun";
    public static final String PHASE1_CHUTE_CATCH = "BR_ChuteCatch";

    public static final String PHASE2_SWITCH_RUN = "BR_2_SwitchRun";
    public static final String PHASE2_CLIMB      = "BR_2_Climb";

    public static final String PHASE3_NONE = "None";

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
    // MULTI-PHASE AUTO BUILDER
    // =========================================================================

    public Command buildAuto(String phase1, String phase2, String phase3) {
        return buildPhase1(phase1)
            .andThen(buildPhase2(phase2))
            .andThen(buildPhase3(phase3));
    }

    private Command buildPhase1(String selection) {
        switch (selection) {
            case PHASE1_CENTER_RUN:  return phase1_CenterRun();
            case PHASE1_CHUTE_CATCH: return phase1_ChuteCatch();
            default:                 return Commands.none();
        }
    }

    private Command buildPhase2(String selection) {
        switch (selection) {
            case PHASE2_SWITCH_RUN: return phase2_SwitchRun();
            case PHASE2_CLIMB:      return phase2_Climb();
            default:                return Commands.none();
        }
    }

    private Command buildPhase3(String selection) {
        return Commands.none();
    }

    // =========================================================================
    // SINGLE-AUTO ENTRY POINT (used by AutoChooser in RobotContainer)
    // =========================================================================

    public void singleCenterShootAuto(AutoRoutine routine) {
        AutoTrajectory path = routine.trajectory("BR_CenterRun");

        routine.active().onTrue(
            path.resetOdometry()
                .andThen(
                    path.cmd().alongWith(
                        Commands.runOnce(m_shooter::closeShot, m_shooter)
                    )
                )
                .andThen(
                    new AlignToHubCommand(m_drivetrain, m_shooter, m_indexer, m_vision, 1.5)
                )
        );

        path.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());
    }

    // =========================================================================
    // PHASE 1 ROUTINES
    // =========================================================================

    private Command phase1_CenterRun() {
        final AutoRoutine routine = m_factory.newRoutine("BR_CenterRun");
        final AutoTrajectory path = routine.trajectory("BR_CenterRun");

        routine.active().onTrue(
            path.resetOdometry()
                .andThen(
                    path.cmd().alongWith(
                        Commands.runOnce(m_shooter::closeShot, m_shooter)
                    )
                )
                .andThen(
                    new AlignToHubCommand(m_drivetrain, m_shooter, m_indexer, m_vision, 1.5)
                )
        );

        path.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());

        return routine.cmd();
    }

    private Command phase1_ChuteCatch() {
        // TODO: implement once BR_ChuteCatch.traj is ready
        return Commands.none();
    }

    // =========================================================================
    // PHASE 2 ROUTINES
    // =========================================================================

    private Command phase2_SwitchRun() {
        final AutoRoutine routine = m_factory.newRoutine("BR_2_SwitchRun");
        final AutoTrajectory path = routine.trajectory("BR_2_SwitchRun");

        routine.active().onTrue(path.cmd());

        path.atTime("StartIntake").onTrue(m_intake.intakeFuel(8.0));
        path.atTime("StopIntake").onTrue(m_intake.stopFuel());

        return routine.cmd();
    }

    private Command phase2_Climb() {
        final AutoRoutine routine = m_factory.newRoutine("BR_2_Climb");
        final AutoTrajectory path = routine.trajectory("BR_2_Climb");

        routine.active().onTrue(path.cmd());

        path.atTime("RetractIntake").onTrue(m_intake.stopFuel());

        return routine.cmd();
    }
}