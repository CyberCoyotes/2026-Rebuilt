package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.commands.FuelCommands;

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

        public static final double shootTimeout = 5.0; // seconds
        public static final double intakeTimeout = 6.0; // seconds

        // ============================================================================
        // Main Auto Routines
        // ============================================================================
                
        // Right Trench to Middle to Ramp Shot
                public AutoRoutine RtTrench_Ramp_Single() {
                
                final AutoRoutine routine = m_factory.newRoutine("Rt Single Trench-Ramp");

                // Trajectories
                final AutoTrajectory RtTrench_Middle = routine.trajectory("RtTrench_Middle", 0);
                final AutoTrajectory RtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
                final AutoTrajectory RtShootRamp = routine.trajectory("RtShootRamp", 0);                        
                
                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Middle.resetOdometry(),
                                                
                                                // 1. Trench to Middle, setup for Ramp Crossing
                                                RtTrench_Middle.cmd(),

                                                // 2. Ramp crossing
                                                RtRampMiddle_Alliance.cmd(),
                                                
                                                // 3. Shoot
                                                RtShootRamp.cmd()

                                                )

                                );
                // Routine Events
                RtTrench_Middle.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout));
                RtShootRamp.atTime("Shoot").onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout));

                return routine;
        }
                // Right Trench to Middle to Ramp Shot
                public AutoRoutine RtTrench_Ramp_Double() {

                final AutoRoutine routine = m_factory.newRoutine("Rt x2 Trench-Ramp");

                // Trajectories
                final AutoTrajectory RtTrench_Middle = routine.trajectory("RtTrench_Middle", 0);
                final AutoTrajectory RtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
                final AutoTrajectory RtShootRamp = routine.trajectory("RtShootRamp", 0);    

                final AutoTrajectory RtShootRamp_Trench = routine.trajectory("RtShootRamp_Trench", 0);
                // final AutoTrajectory RtTrench_CurlSweep = routine.trajectory("RtTrench_CurlSweep", 0);

                /* Probably not needed?
                final AutoTrajectory RtRampMiddle_RtRampAlliance2 = routine.trajectory("RtRampMiddle_RtRampAlliance", 0);
                final AutoTrajectory RtShot2 = routine.trajectory("RtRampAlli_Shot", 0);     
                */

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Middle.resetOdometry(),
                                                
                                                 // 1. Trench to Middle, setup for Ramp Crossing
                                                RtTrench_Middle.cmd(),

                                                 // 2. Ramp crossing to Alliance side
                                                RtRampMiddle_Alliance.cmd(),

                                                // 3. Shoot
                                                RtShootRamp.cmd(),

                                                // 4. Back to trench
                                                RtShootRamp_Trench.cmd(),

                                                // 5. Trench to Middle, setup for Ramp Crossing (2nd)
                                                RtTrench_Middle.cmd(),

                                                // 6. Ramp crossing to Alliance side (2nd)
                                                RtRampMiddle_Alliance.cmd(),

                                                // 7. Shoot (2nd)
                                                RtShootRamp.cmd() 
                                                )

                                );
                // Routine Events
                RtTrench_Middle.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout));
                RtShootRamp.atTime("Shoot").onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout));
                // RtTr_CurlSweep.atTime("Intake").onTrue(m_intake.intakeFuelTimer(6));
                // RtRampAlli_Shot2.atTime("Shoot").onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout));
                // LtTrench_Mid_Trench.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer));

                return routine;
        }
        

        


        // Left Trench to Middle to Ramp Shot
        public AutoRoutine LtTrench_Ramp_Single() {

                final AutoRoutine routine = m_factory.newRoutine("Lt Single Trench-Ramp");

                final AutoTrajectory LtTr_LtMid = routine.trajectory("LtTrench_Middle", 0);
                
                // LeftRampAlign to LeftRampShot
                final AutoTrajectory LtRampMid_LtRampAlli = routine.trajectory("LtRampMid_LtRampAlli", 0);
                
                // LeftRampShoot to LeftTrench
                final AutoTrajectory LtRampAlli_Shot = routine.trajectory("LtRampAlli_Shot", 0); 

                // LeftTrench to LeftSweep to LeftRampShot
                // final AutoTrajectory LtTr_LtSweep = routine.trajectory("LtTr_LtSweep", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTr_LtMid.resetOdometry(), // Always reset odometry first
                                                
                                                LtTr_LtMid.cmd(),
                                                
                                                LtRampMid_LtRampAlli.cmd(),

                                                LtRampAlli_Shot.cmd()

                                ));

                // Routine Events
                LtTr_LtMid.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout));

                LtRampAlli_Shot.atTime("Shoot").onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake,m_drivetrain, shootTimeout));

                return routine;
        }

        
        // Left Trench to Middle to Ramp Shot
        public AutoRoutine LtTrench_Ramp_Double() {

                final AutoRoutine routine = m_factory.newRoutine("Lt x2 Trench-Ramp");
                // LeftTrench to LeftMiddle to LeftRampAlign
                final AutoTrajectory LtTr_LtMid = routine.trajectory("LtTr_LtMid", 0);
                
                // LeftRampAlign to LeftRampShot
                final AutoTrajectory LtRampMid_LtRampAlli = routine.trajectory("LtRampMid_LtRampAlli", 0);
                
                // LeftRampShoot to LeftTrench
                final AutoTrajectory LtRampAlli_Shot = routine.trajectory("LtRampAlli_Shot", 0); 

                // TODO Confirm final AutoTrajectory LtShot_Trench = routine.trajectory("LtShot_Trench", 0);

                // TODO Confirm final AutoTrajectory LtTr_CurlSweep = routine.trajectory("LtTr_CurlSweep", 0);

                // LeftTrench to LeftSweep to LeftRampShot
                // final AutoTrajectory LtTr_LtSweep = routine.trajectory("LtTr_LtSweep", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTr_LtMid.resetOdometry(), // Always reset odometry first
                                                
                                                LtTr_LtMid.cmd(),
                                                
                                                LtRampMid_LtRampAlli.cmd(),

                                                LtRampAlli_Shot.cmd()

                                                // Back to trench, intake first
                                                // TODO Confirm LtShot_Trench.cmd(),

                                                // Sweep out second time
                                                // TODO ConfirmLtTr_CurlSweep.cmd(),

                                                // Come across 2nd time
                                                // LtRampMid_RtRampAlli.cmd(), 

                                                // Shoot 2nd time
                                                // LtRampAlli_Shot.cmd()

                                ));

                // Routine Events
                LtTr_LtMid.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout));

                LtRampAlli_Shot.atTime("Shoot").onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake,m_drivetrain, shootTimeout));

                return routine;
        }

        // ============================================================================
        // Secondary Auto Routines
        // ============================================================================
        



        


        // ============================================================================
        // Experimental
        // ============================================================================

        // Inner class for testing new auto features without affecting existing routines
        public class Test {
                        public AutoRoutine Bulldozer() {
                final AutoRoutine routine = m_factory.newRoutine("Bulldozer 2026");
                final AutoTrajectory Bulldozer = routine.trajectory("Bulldozer2026", 0);

                final AutoTrajectory RtRampMid_RtRampAlli = routine.trajectory("RtRampMid_RtRampAlli", 0);
                
                final AutoTrajectory RtRampAlli_Shot = routine.trajectory("RtRampAlli_Shot", 0);                        
                
                // final AutoTrajectory RtShot_Trench = routine.trajectory("RtShot_Trench", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                Bulldozer.resetOdometry(),
                                                
                                                Bulldozer.cmd(),
                                                
                                                RtRampMid_RtRampAlli.cmd(),

                                                RtRampAlli_Shot.cmd()
                                                

                                ));
                // Routine Events
                Bulldozer.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout + 2));

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
                RtTrench_Mid_Ramp.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout));
                RtTrench_Mid_Ramp.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer,m_intake, m_drivetrain, shootTimeout));
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
                LtTrench_Mid_Trench.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout));
                LtTrench_Mid_Trench.atTime("Shoot").onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout));
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
                MidDepot.atTime("Shoot").onTrue(FuelCommands.Auto.shootFar(m_shooter, m_indexer, shootTimeout)); // score

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
                                .onTrue(FuelCommands.Auto.shootHub(m_shooter, m_indexer,shootTimeout));
                // Center.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer));

                return routine;
        }

                public AutoRoutine RtTrench_RtMid_RtTrench() {
                final AutoRoutine routine = m_factory.newRoutine("RtTrench_RtMid_RtTrench");
                final AutoTrajectory RtTrench_RtMid_RtTrench = routine.trajectory("RtTrench_RtMid_RtTrench", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_RtMid_RtTrench.resetOdometry(), // Always reset odometry first
                                                RtTrench_RtMid_RtTrench.cmd() // Follow the path

                                ));
                // Routine Events
                RtTrench_RtMid_RtTrench.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout));
                
                // dependencies are fine in this version
                RtTrench_RtMid_RtTrench.atTime("Shoot")
                                .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout));
                RtTrench_RtMid_RtTrench.atTime("FuelPump").onTrue(FuelCommands.Auto.fuelPumpCycleSensor(m_intake, m_indexer));
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
                                        .onTrue(FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake,m_drivetrain,
                                                        shootTimeout));

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