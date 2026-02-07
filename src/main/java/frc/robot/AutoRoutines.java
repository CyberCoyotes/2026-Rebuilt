package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@SuppressWarnings("unused")

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    /*private final ClimberSubsystem m_climber;
    private final IndexerSubsystem m_indexer;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;*/

    // How long to wait after driving before doing something else
    private final double DRIVE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less 

    private final double SCORE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less 

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain/* , ClimberSubsystem climber,
            IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter*/) {
        m_factory = factory;
        m_drivetrain = drivetrain;
       /*  m_climber = climber;
        m_indexer = indexer;
        m_intake = intake;
        m_shooter = shooter;*/
    }

    public AutoRoutine FM() {
                 final AutoRoutine routine = m_factory.newRoutine("FourMeters");
                final AutoTrajectory FM = routine.trajectory("FourMeters", 0);
               // final AutoTrajectory FM2 = routine.trajectory("FourMeters", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                FM.resetOdometry(), // Always reset odometry first
                                FM.cmd()//, // Follow the path
                                //m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                               // FM2.cmd()

                        ));
                //.atTime("Score").onTrue(m_indexerCommands.autoScore()); //score

                // Consider using m_commandGroups.autoIntakeCoral(m_indexerCommands, m_shooterCommands,/*m_wrist*/);
                // .atTime("Load").onTrue(m_intakeCommands.intake());
                return routine;
        }
         public AutoRoutine B() {
                 final AutoRoutine routine = m_factory.newRoutine("Basic");
                final AutoTrajectory B = routine.trajectory("Basic", 0);
               // final AutoTrajectory B2 = routine.trajectory("Basic", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                B.resetOdometry(), // Always reset odometry first
                                B.cmd()//, // Follow the path
                                //m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                               // FM2.cmd()

                        ));
                //.atTime("Score").onTrue(m_indexerCommands.autoScore()); //score

                // Consider using m_commandGroups.autoIntakeCoral(m_indexerCommands, m_shooterCommands,/*m_wrist*/);
                // .atTime("Load").onTrue(m_intakeCommands.intake());
                return routine;
        }
}
