package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.commands.FuelCommands;
import frc.robot.commands.FuelCommands.Auto;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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



        public AutoRoutine RtTrench_Mid_Trench() {
                final AutoRoutine routine = m_factory.newRoutine("RtTrench_Mid_Trench");
                final AutoTrajectory RtTrench_Mid_Trench = routine.trajectory("RtTrench_Mid_Trench", 0);
                // final AutoTrajectory StartRMid2 = routine.trajectory("StartRMid", 1);
                // final AutoTrajectory StartRMid3 = routine.trajectory("StartRMid", 2);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Mid_Trench.resetOdometry(), // Always reset odometry first
                                                RtTrench_Mid_Trench.cmd() // Follow the path
                                // m_drivetrain.stop().withTimeout(3),
                                // StartRMid2.cmd(),
                                // StartRMid3.cmd()

                                ));
                // Routine Events
                RtTrench_Mid_Trench.atTime("Intake").onTrue(m_intake.intakeFuelTimer(5));
                // Option A — fixed timer:
                // RtTrench_Mid_Trench.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleAuto(2.0));
                // Option B — sensor-gated (waits for first detection, stops when chute clears):
                // RtTrench_Mid_Trench.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleSensor(m_indexer));
                // Vision Shot
                RtTrench_Mid_Trench.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0));
                RtTrench_Mid_Trench.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleSensor(m_indexer));

                return routine;
        }

        public AutoRoutine RtTrench_Mid_Trench_Splits() {
                final AutoRoutine routine = m_factory.newRoutine("RtTrench_Mid_Trench_Splits");
                final AutoTrajectory RtTrench_Mid = routine.trajectory("RtTrench_Mid", 0);
                final AutoTrajectory RtMid_Trench = routine.trajectory("RtMid_Trench", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Mid.resetOdometry(), // Always reset odometry first
                                                RtTrench_Mid.cmd(), // Follow the path
                                                RtMid_Trench.cmd() // Follow the path

                                ));
                // Routine Events
                RtTrench_Mid.atTime("Intake").onTrue(m_intake.intakeFuelTimer(5));
                // Option A — fixed timer:
                // RtTrench_Mid_Trench.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleAuto(2.0));
                // Option B — sensor-gated (waits for first detection, stops when chute clears):
                // RtTrench_Mid_Trench.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleSensor(m_indexer));
                // Vision Shot
                RtMid_Trench.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0));
                RtMid_Trench.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleSensor(m_indexer));

                return routine;
        }

        public AutoRoutine RtTrench_Mid_Ramp() {
                final AutoRoutine routine = m_factory.newRoutine("RtTrench_Mid_Ramp");
                final AutoTrajectory RtTrench_Mid_Ramp = routine.trajectory("RtTrench_Mid_Ramp", 0);
                // final AutoTrajectory StartRMid2 = routine.trajectory("StartRMid", 1);
                // final AutoTrajectory StartRMid3 = routine.trajectory("StartRMid", 2);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Mid_Ramp.resetOdometry(), // Always reset odometry first
                                                RtTrench_Mid_Ramp.cmd() // Follow the path
                                // m_drivetrain.stop().withTimeout(3),
                                // StartRMid2.cmd(),
                                // StartRMid3.cmd()

                                ));
                // Routine Events
                RtTrench_Mid_Ramp.atTime("Intake").onTrue(m_intake.intakeFuelTimer(5));
                // Option A — fixed timer:
                // RtTrench_Mid_Trench.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleAuto(2.0));
                // Option B — sensor-gated (waits for first detection, stops when chute clears):
                // RtTrench_Mid_Ramp.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleSensor(m_indexer));
                // Vision Shot
                RtTrench_Mid_Ramp.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0));
                RtTrench_Mid_Ramp.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleSensor(m_indexer));

                return routine;
        }

        public AutoRoutine LtTrench_Mid_Trench() {
                final AutoRoutine routine = m_factory.newRoutine("LtTrench_Mid_Trench");
                final AutoTrajectory LtTrench_Mid_Trench = routine.trajectory("LtTrench_Mid_Trench", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTrench_Mid_Trench.resetOdometry(), // Always reset odometry first
                                                LtTrench_Mid_Trench.cmd() // Follow the pat

                                ));
                // Routine Events
                LtTrench_Mid_Trench.atTime("Shoot").onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0));
                LtTrench_Mid_Trench.atTime("FuelPump").onTrue(m_intake.fuelPumpCycleSensor(m_indexer));

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

        // Inner class for testing new auto features without affecting existing routines
        public class Test {

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

                public AutoRoutine visionTest() {
                        final AutoRoutine routine = m_factory.newRoutine("VisionTest");
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
                                        .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain,
                                                        3.0));

                        return routine;
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
        } // end of inner class

} // end of class