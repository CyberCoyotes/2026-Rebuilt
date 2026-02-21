package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@SuppressWarnings("unused")

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final IntakeSubsystem m_intake;
    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;
    private final ShooterCommands m_shooterCommands;
   // private final VisionSubsystem m_vision;

  //  private final LedSubsystem m_ledSubsystem;

    // How long to wait after driving before doing something else
    private final double DRIVE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less 

    private final double SCORE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less 

    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain,/* , ClimberSubsystem climber,*/
         IndexerSubsystem indexer,IntakeSubsystem intake, ShooterSubsystem shooter, ShooterCommands shooterCommands/*, VisionSubsystem vision, LedSubsystem ledSubsystem*/) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        //m_climber = climber;
        m_indexer = indexer;
        m_intake = intake;
        m_shooter = shooter;
        m_shooterCommands = shooterCommands;
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
          public AutoRoutine Lob() {
                 final AutoRoutine routine = m_factory.newRoutine("Lob");
                final AutoTrajectory Lob = routine.trajectory("Lob", 0);
               // final AutoTrajectory Lob2 = routine.trajectory("Lob", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                Lob.resetOdometry(), // Always reset odometry first
                                Lob.cmd(),//, // Follow the path
                                m_drivetrain.stop().withTimeout(DRIVE_WAIT)
                               // Lob2.cmd()

                        ));
                Lob.atTime("Score").onTrue(ShooterCommands.rampTestShoot(m_shooter, m_indexer)); //score

                // Consider using m_commandGroups.autoIntakeCoral(m_indexerCommands, m_shooterCommands,/*m_wrist*/);
                //   Lob2.atTime("Load").onTrue(m_intakeCommands.intake());
                return routine;
        }

        public AutoRoutine Default() {
                 final AutoRoutine routine = m_factory.newRoutine("DefaultRightV2");
                final AutoTrajectory DefaultRightV2 = routine.trajectory("DefaultRightV2", 0);
               // final AutoTrajectory DefaultRightV2 = routine.trajectory("DefaultRightV2", 1);

                routine.active().onTrue(
                        Commands.sequence(
                                DefaultRightV2.resetOdometry(), // Always reset odometry first
                                DefaultRightV2.cmd()//, // Follow the path
                                ,m_drivetrain.stop().withTimeout(DRIVE_WAIT)
                               // DefaultRightV2.cmd()

                        ));
                DefaultRightV2.atTime("Score").onTrue(ShooterCommands.rampTestShoot(m_shooter, m_indexer)); //score

                // Consider using m_commandGroups.autoIntakeCoral(m_indexerCommands, m_shooterCommands,/*m_wrist*/);
                //DefaultRightV2.atTime("Load").onTrue(m_intakeCommands.intake());
                return routine;
        }
}
