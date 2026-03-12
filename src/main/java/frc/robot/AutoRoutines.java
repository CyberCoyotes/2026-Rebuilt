package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.commands.FuelCommands;
import frc.robot.commands.FuelCommands.Auto;
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
        private final FuelCommands m_fuelCommands;
        private final VisionSubsystem m_vision;
        // private final LedSubsystem m_ledSubsystem;

        // How long to wait after driving before doing something else
        private final double DRIVE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less
        private final double SCORE_WAIT = 1.0; // Cut 2.0 -> 1.0 or less

        public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain, /* , ClimberSubsystem climber, */
                        IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter,
                        FuelCommands fuelCommands, VisionSubsystem vision/* , LedSubsystem ledSubsystem */) {
                m_factory = factory;
                m_drivetrain = drivetrain;
                // m_climber = climber;
                m_indexer = indexer;
                m_intake = intake;
                m_shooter = shooter;
                m_vision = vision;
                m_fuelCommands = fuelCommands;
        }

        public AutoRoutine FM() {
                final AutoRoutine routine = m_factory.newRoutine("FourMeters");
                final AutoTrajectory FM = routine.trajectory("FourMeters", 0);
                // final AutoTrajectory FM2 = routine.trajectory("FourMeters", 1);

                routine.active().onTrue(
                                Commands.sequence(
                                                FM.resetOdometry(), // Always reset odometry first
                                                FM.cmd()// , // Follow the path
                                // m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                // FM2.cmd()

                                ));
                // Routine Events
                // .atTime("Score").onTrue(m_indexerCommands.autoScore()); //score

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
                                                Lob.cmd(), // , // Follow the path
                                                m_drivetrain.stop().withTimeout(DRIVE_WAIT)
                                // Lob2.cmd()

                                ));
                // Routine Events
                // Lob.atTime("Shoot").onTrue(ShooterCommands.shootAtCurrentTarget(m_shooter,
                // m_indexer)); //score

                // Consider using m_commandGroups.autoIntakeCoral(m_indexerCommands,
                // m_shooterCommands,/*m_wrist*/);
                // Lob2.atTime("Load").onTrue(m_intakeCommands.intake());
                return routine;
        }

        public AutoRoutine StartRMid() {
                final AutoRoutine routine = m_factory.newRoutine("StartRMid");
                final AutoTrajectory StartRMid = routine.trajectory("StartRMid", 0);
                // final AutoTrajectory StartRMid2 = routine.trajectory("StartRMid", 1);
                // final AutoTrajectory StartRMid3 = routine.trajectory("StartRMid", 2);

                routine.active().onTrue(
                                Commands.sequence(
                                                StartRMid.resetOdometry(), // Always reset odometry first
                                                StartRMid.cmd() // Follow the path
                                // m_drivetrain.stop().withTimeout(3),
                                // StartRMid2.cmd(),
                                // StartRMid3.cmd()

                                ));
                // Routine Events
                StartRMid.atTime("Intake").onTrue(m_intake.intakeFuelTimer(10));

                StartRMid.atTime("Shoot").onTrue(FuelCommands.Auto.shootTrench(m_shooter, m_indexer, 6)); // FIXME:
                                                                                                          // Check
                                                                                                          // segement
                                                                                                          // number
                // Stay in a line! Color in the lines

                // DefaultRightV2.atTime("Load").onTrue(m_intakeCommands.intake());
                return routine;
        }

        public AutoRoutine StartLMid() {
                final AutoRoutine routine = m_factory.newRoutine("StartLMid");
                final AutoTrajectory StartLMid = routine.trajectory("StartLMid", 0);
                // final AutoTrajectory StartLMid2 = routine.trajectory("StartRMid", 1);
                // final AutoTrajectory StartLMid3 = routine.trajectory("StartRMid", 2);

                routine.active().onTrue(
                                Commands.sequence(
                                                StartLMid.resetOdometry(), // Always reset odometry first
                                                StartLMid.cmd() // Follow the path
                                // m_drivetrain.stop().withTimeout(3),
                                // StartRMid2.cmd(),
                                // StartRMid3.cmd()

                                ));
                // Routine Events
                StartLMid.atTime("Intake").onTrue(m_intake.intakeFuelTimer(10));

                StartLMid.atTime("Shoot").onTrue(FuelCommands.Auto.shootTrench(m_shooter, m_indexer, 6)); // FIXME:
                                                                                                          // Check
                                                                                                          // segement
                                                                                                          // number
                // Stay in a line! Color in the lines

                // DefaultRightV2.atTime("Load").onTrue(m_intakeCommands.intake());
                return routine;
        }

        public AutoRoutine TestRoutine() {
                final AutoRoutine routine = m_factory.newRoutine("TestRoutine");
                final AutoTrajectory TestRoutine = routine.trajectory("TestRoutine", 0);
                final AutoTrajectory TestRountine2 = routine.trajectory("TestRoutine", 1);

                routine.active().onTrue(
                                Commands.sequence(
                                                TestRoutine.resetOdometry(), // Always reset odometry first
                                                TestRoutine.cmd(), // Follow the path
                                                m_drivetrain.stop().withTimeout(10.0),
                                                TestRountine2.cmd()

                                ));
                // Routine Events

                TestRoutine.atTime("Shoot").onTrue(FuelCommands.Auto.shootHub(m_shooter, m_indexer, 3.0)); // score
                // Dummy.atTime("Intake").onTrue(IntakeSubsystem.intakeFuel(m_shooter,
                // m_indexer)); //score

                return routine;
        }

        public AutoRoutine MidDepot() {
                final AutoRoutine routine = m_factory.newRoutine("MidDepot");
                final AutoTrajectory MidDepot = routine.trajectory("MidDepot", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                MidDepot.resetOdometry(), // Always reset odometry first
                                                MidDepot.cmd() // Follow the path

                                ));
                // Routine Events
                MidDepot.atTime("Intake").onTrue(m_intake.intakeFuelTimer(10));
                MidDepot.atTime("Shoot").onTrue(FuelCommands.Auto.shootFar(m_shooter, m_indexer, 6)); // score

                return routine;
        }

        // ============================================================================
        // Experimental
        // ============================================================================

        public AutoRoutine visionTest01() {
                final AutoRoutine routine = m_factory.newRoutine("VisionTest01");
                final AutoTrajectory Experimental = routine.trajectory("VisionTest", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                Experimental.resetOdometry(), // Always reset odometry first
                                                Experimental.cmd() // Follow the path

                                ));
                // Routine Events

                // Odometry-based align and shoot — no vision latency, no driver input, ends on
                // its own.
                // Place the "Shoot" event marker at the END of the trajectory segment so the
                // path finishes before this fires.
                Experimental.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 1.0));

                return routine;
        }

        // FIXME This was not written correctly and I don't have time to fix it.
        // public AutoRoutine visionTest02() {
        // final AutoRoutine routine = m_factory.newRoutine("VisionTest02");
        // final AutoTrajectory Experimental = routine.trajectory("VisionTest", 0);

        // routine.active().onTrue(
        // Commands.sequence(
        // Experimental.resetOdometry(), // Always reset odometry first
        // Experimental.cmd() //Follow the path

        // ));
        // // Routine Events
        // Experimental.atTime("Shoot").onTrue(FuelCommands.Auto.visionShot_version2(m_shooter,
        // m_vision, m_indexer, m_drivetrain)); //score

        // return routine;
        // }
}
