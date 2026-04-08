package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FuelCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoRoutines {
        private final AutoFactory m_factory;
        private final CommandSwerveDrivetrain m_drivetrain;
        private final IntakeSubsystem m_intake;
        private final IndexerSubsystem m_indexer;
        private final ShooterSubsystem m_shooter;

        public AutoRoutines(AutoFactory factory, 
                        CommandSwerveDrivetrain drivetrain,
                        IndexerSubsystem indexer, 
                        IntakeSubsystem intake, 
                        ShooterSubsystem shooter) {
                m_factory = factory;
                m_drivetrain = drivetrain;
                m_indexer = indexer;
                m_intake = intake;
                m_shooter = shooter;
                // m_vision = vision;
        }

        // FIXME: This routine needs validation
        public AutoRoutine RtTrench_RtMid_RtTrench() {
                final AutoRoutine routine = m_factory.newRoutine("RtTrench_RtMid_RtTrench");
                final AutoTrajectory RtTrench_RtMid_RtTrench = routine.trajectory("RtTrench_RtMid_RtTrench", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_RtMid_RtTrench.resetOdometry(), // Always reset odometry first
                                                RtTrench_RtMid_RtTrench.cmd() // Follow the path

                                ));
                // Routine Events
                RtTrench_RtMid_RtTrench.atTime("Intake").onTrue(m_intake.intakeFuelTimer(6));

                RtTrench_RtMid_RtTrench.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0));
                RtTrench_RtMid_RtTrench.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer));
                return routine;
        }

// FIXME: Validate this routine 1st
                // Right Trench to Middle to Ramp Shot
                public AutoRoutine RtTrench_Ramp_Double() {

                final AutoRoutine routine = m_factory.newRoutine("Rt x2 Trench-Ramp");

                // RightTrench to RightMiddle to RightRampAlign
                final AutoTrajectory RtTr_RtMid = routine.trajectory("RtTr_RtMid", 0);
                
                // RightRampAlign to RightRampShot
                // final AutoTrajectory RtMid_RtRampShot = routine.trajectory("RtMid_RtRampShot", 0);
                
                final AutoTrajectory RtRampMid_RtRampZone = routine.trajectory("RtRampMid_RtRampZone", 0);
                
                // RightRampShoot to RightTrench
                final AutoTrajectory RtRampZone_RtTr = routine.trajectory("RtRampZone_RtTr", 0);                        
                
                // RightTrench to RightSweep to RightRampShot
                final AutoTrajectory RtTr_RtCurlSweep = routine.trajectory("RtTr_RtCurlSweep", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTr_RtMid.resetOdometry(), // Always reset odometry first

                                                RtTr_RtMid.cmd(), // 3.6 seconds                                                
                                                RtRampMid_RtRampZone.cmd(),
                                                // TODO: Test and tune this shooting + pumping sequence
                                                // TODO: Measure time to unload in Auton. It will vary depending on the number of balls, but measuring should give a better estimate
                                                // Commands.parallel(
                                                        
                                                //         /* TODO: Not sure if this will end on it's own or rely on the safety timeout. 
                                                //         * One possible fix is the CHUTE_SENSOR or literally integrate the fuelPumpCycleSensor() into the autonomous Shooting
                                                //         */
                                                //         FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0),

                                                //         // TODO: Test fuel pump cycle sensor and if it ends on its own, based on sensor. 
                                                //         FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer) 
                                                // ), // Approximately 4.0 seconds total for alignment + shooting + pumping
                                                RtRampZone_RtTr.cmd(), // 1.2 seconds
                                                
                                                RtTr_RtCurlSweep.cmd() // 2.0 seconds
                                                
                                                // RtMid_RtRampShot.cmd(), // 1.6 seconds
                                                
                                                // Commands.parallel( 
                                                //         FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0), 
                                                //         FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer)
                                                ) // Approximately 4.0 seconds total for alignment + shooting + pumping

                                                // ---------------------- 10.0 seconds total (est) without shooting ----------------------
                                                // ----------------------- 18.0 seconds total (est) with shooting ----------------------


                                );
                // Routine Events
                RtTr_RtMid.atTime("Intake").onTrue(m_intake.intakeFuelTimer(6));

                return routine;
        }
        
        // Left Trench to Middle to Ramp Shot
        public AutoRoutine LtTrench_Ramp_Double() {

                final AutoRoutine routine = m_factory.newRoutine("Lt x2 Trench-Ramp");
                // LeftTrench to LeftMiddle to LeftRampAlign
                final AutoTrajectory LtTr_LtMid = routine.trajectory("LtTr_LtMid", 0);
                // LeftRampAlign to LeftRampShot
                final AutoTrajectory LtMid_LtRampShot = routine.trajectory("LtMid_LtRampShot", 0);
                // LeftRampShoot to LeftTrench
                final AutoTrajectory LtRampShot_LtTr = routine.trajectory("LtRampShot_LtTr", 0);                        
                // LeftTrench to LeftSweep to LeftRampShot
                final AutoTrajectory LtTr_LtSweep = routine.trajectory("LtTr_LtSweep", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTr_LtMid.resetOdometry(), // Always reset odometry first
                                                
                                                LtTr_LtMid.cmd(), // 3.6 seconds
                                                
                                                LtMid_LtRampShot.cmd(), // 1.6 seconds

                                                // TODO: Add shoot after confirming with Liam and testing right side. 
                                                // If the timing is already tight, we may need to optimize the path or reduce the wait time after driving to fit it in.

                                                LtRampShot_LtTr.cmd(), // 1.2 seconds
                                                
                                                LtTr_LtSweep.cmd(), // 2.0 seconds
                                                
                                                LtMid_LtRampShot.cmd() // 1.6 seconds

                                                // TODO: Add shooting

                                                // ---------------------- 10.0 seconds total (est) without shooting ----------------------


                                ));
                // Routine Events
                LtTr_LtMid.atTime("Intake").onTrue(m_intake.intakeFuelTimer(6));

                return routine;
        }

        // FIXME: This routine needs validation
        public AutoRoutine Full_RtTrench_Mid_Ramp() {
                final AutoRoutine routine = m_factory.newRoutine("FULL RtTrench_Mid_Ramp");
                final AutoTrajectory RtTrench_Mid_Ramp = routine.trajectory("Full_RtTrench_Mid_Ramp", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Mid_Ramp.resetOdometry(), // Always reset odometry first
                                                RtTrench_Mid_Ramp.cmd() // Follow the path
                                // m_drivetrain.stop().withTimeout(3),


                                ));
                // Routine Events
                RtTrench_Mid_Ramp.atTime("Intake").onTrue(m_intake.intakeFuelTimer(6));
                RtTrench_Mid_Ramp.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0));
                RtTrench_Mid_Ramp.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer));

                return routine;
        }

        // FIXME: This routine is currently broken in Choreo
        public AutoRoutine Full_LtTrench_Mid_Trench() {
                final AutoRoutine routine = m_factory.newRoutine("FULL Lt Trench-Mid-Trench");
                final AutoTrajectory LtTrench_Mid_Trench = routine.trajectory("Full_LtTrench_Mid_Trench", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTrench_Mid_Trench.resetOdometry(), // Always reset odometry first
                                                LtTrench_Mid_Trench.cmd() // Follow the path

                                ));
                // Routine Events
                LtTrench_Mid_Trench.atTime("Intake").onTrue(m_intake.intakeFuelTimer(8));
                LtTrench_Mid_Trench.atTime("Shoot").onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_drivetrain, 6.0));
                LtTrench_Mid_Trench.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer));

                return routine;
        }

        public AutoRoutine MidDepot() {
                final AutoRoutine routine = m_factory.newRoutine("MidDepot");
                final AutoTrajectory MidDepot = routine.trajectory("MidDepot", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                MidDepot.resetOdometry(), // Always reset odometry first
                                                MidDepot.cmd() // Follow the path

                                                // TODO: Add shooting after confirming path

                                ));
                // Routine Events
                MidDepot.atTime("Intake").onTrue(m_intake.intakeFuelTimer(8));
                MidDepot.atTime("Shoot").onTrue(FuelCommands.Auto.shootFar(m_shooter, m_indexer, 6)); // score

                return routine;
        }

         public AutoRoutine Center() {
                        final AutoRoutine routine = m_factory.newRoutine("Center");
                        final AutoTrajectory Center = routine.trajectory("Center", 0);
                        // final AutoTrajectory TestRountine2 = routine.trajectory("Center", 1);

                        routine.active().onTrue(
                                        Commands.sequence(
                                                        Center.resetOdometry(), // Always reset odometry first
                                                        Center.cmd(), // Follow the path
                                                        
                                                        // TODO: Add shooting command

                                                        m_drivetrain.stop().withTimeout(10.0)
                                                        

                                        ));
                        // Routine Events

                Center.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.shootHub(m_shooter, m_indexer,6.6));
                // Center.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer));

                return routine;
        }

        public AutoRoutine Bulldozer() {
                final AutoRoutine routine = m_factory.newRoutine("Bulldozer 2026");
                final AutoTrajectory Bulldozer = routine.trajectory("Bulldozer2026", 0);
                final AutoTrajectory RtRamp_RtRampShot = routine.trajectory("RtRamp_RtRampShot", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                Bulldozer.resetOdometry(),
                                                Bulldozer.cmd(),
                                                RtRamp_RtRampShot.cmd()

                                                // TODO Add shooting after confirming path

                                ));
                // Routine Events
                Bulldozer.atTime("Intake").onTrue(m_intake.intakeFuelTimer(8));

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
                                                        m_drivetrain.stop().withTimeout(Constants.Auto.DRIVE_WAIT)
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

                // ============================================================================
                // Example routine from previous season with multiple trajectories and events of different types, 
                // for testing and demonstration purposes. circa 2025
                // @lvanscoyoc  A.K.A. "Liam GOAT"
                // ============================================================================

                public AutoRoutine STAtoL() {
                        final AutoRoutine routine = m_factory.newRoutine("ST-A");
                        final AutoTrajectory STA = routine.trajectory("ST-A", 0);
                        final AutoTrajectory STA2 = routine.trajectory("ST-A", 1);
                        final AutoTrajectory CSL = routine.trajectory("CS1-L", 0);
                        final AutoTrajectory CSL2 = routine.trajectory("CS1-L", 1);

                        routine.active().onTrue(
                                        Commands.sequence(
                                                        STA.resetOdometry(), // Always reset odometry first
                                                        STA.cmd(), // Follow the path
                                                        // m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                                        STA2.cmd(),
                                                        // m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                                        CSL.cmd(),
                                                        // m_drivetrain.stop().withTimeout(DRIVE_WAIT),
                                                        CSL2.cmd()

                                        ));

                        return routine;
                }
        } // end of inner class

} // end of class