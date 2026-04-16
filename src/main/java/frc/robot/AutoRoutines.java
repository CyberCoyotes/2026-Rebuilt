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
        public static final double fuelPumpTimeout = 4.0; // seconds; adjust as needed based on testing
        public static final double purgeTimeout = 3.0; // seconds to purge fuel
        public static final double intakeDelay = 0.75; //

        // ============================================================================
        // RIGHT SIDE AUTOS - start in right trench
        // ============================================================================
                
        // Right Trench to Middle to Ramp Shot
                public AutoRoutine RtTrench_Ramp_Single() {

                final AutoRoutine routine = m_factory.newRoutine("Right x1 Trench-Ramp");

                // Trajectories
                final AutoTrajectory RtTrench_Middle = routine.trajectory("RtTrench_Middle", 0);
                final AutoTrajectory RtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
                final AutoTrajectory RtShootRamp = routine.trajectory("RtShootRamp", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Middle.resetOdometry(),

                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                RtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing
                                                RtRampMiddle_Alliance.cmd(),

                                                // 3. Drive to shoot position
                                                RtShootRamp.cmd(),

                                                // 4. Shoot — starts only after RtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout)
                                                )
                                );

                return routine;
        }
               // Right Trench to Middle to Ramp Shot
                public AutoRoutine RtTrench_Ramp_Double() {

                final AutoRoutine routine = m_factory.newRoutine("Right x2 Trench-Ramp");

                // Cycle 1
                final AutoTrajectory RtTrench_Middle = routine.trajectory("RtTrench_Middle", 0);
                final AutoTrajectory RtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
                final AutoTrajectory RtShootRamp_Trench = routine.trajectory("RtShootRamp_Trench", 0);

                // Cycle 2 (fresh instances; reusing the same AutoTrajectory object
                // causes its event triggers to not re-fire on the second activation)
                final AutoTrajectory RtTrench_Middle_2 = routine.trajectory("RtTrench_Middle", 0);
                final AutoTrajectory RtRampMiddle_Alliance_2 = routine.trajectory("RtRampMiddle_Alliance", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Middle.resetOdometry(),

                                                // --- Cycle 1 ---
                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                RtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing to Alliance side
                                                RtRampMiddle_Alliance.cmd(),

                                                // 4. Shoot — starts only after RtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout),

                                                // --- Cycle 2 ---
                                                // 5. Back to trench
                                                RtShootRamp_Trench.cmd(),

                                                // 6. Trench to Middle (2nd) — intake runs in parallel
                                                Commands.deadline(
                                                                RtTrench_Middle_2.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 7. Ramp crossing to Alliance side (2nd)
                                                RtRampMiddle_Alliance_2.cmd(),

                                                // 9. Shoot (2nd) — starts only after RtShootRamp_2 fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout)
                                ));

                return routine;
        }

                // Right Trench to Middle to Ramp Shot
                public AutoRoutine RtTrench_Ramp_HubSweep() {

                final AutoRoutine routine = m_factory.newRoutine("Right Trench-Ramp-HubSweep");

                // Cycle 1
                final AutoTrajectory RtTrench_Middle = routine.trajectory("RtTrench_Middle", 0); // + Intake
                final AutoTrajectory RtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
                // Shoot
                final AutoTrajectory RtShootRamp_Trench = routine.trajectory("RtShootRamp_Trench", 0);

                // Cycle 2 (fresh instances; reusing the same AutoTrajectory object
                // causes its event triggers to not re-fire on the second activation)
                final AutoTrajectory RtTrench_HubSweep = routine.trajectory("RtTrench_HubSweep", 0);
                final AutoTrajectory RtHub_Ramp = routine.trajectory("RtHub_Ramp", 0);
                final AutoTrajectory RtRampMiddle_Alliance_2 = routine.trajectory("RtRampMiddle_Alliance", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Middle.resetOdometry(),

                                                // --- Cycle 1 ---
                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                RtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing to Alliance side
                                                RtRampMiddle_Alliance.cmd(),

                                                // 4. Shoot — starts only after RtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout),

                                                // --- Cycle 2 ---
                                                // 5. Back to trench
                                                RtShootRamp_Trench.cmd(),

                                                // 6. Trench to Middle (2nd) — intake runs in parallel
                                                Commands.deadline(
                                                                RtTrench_HubSweep.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                RtHub_Ramp.cmd(),

                                                // 7. Ramp crossing to Alliance side (2nd)
                                                RtRampMiddle_Alliance_2.cmd(),

                                                // 9. Shoot (2nd) — starts only after RtShootRamp_2 fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout)
                                ));

                return routine;
         }

         /* CYCLE 1: Run to the middle - deploy intake shortly after start - and collect fuel. 
         * It will line up and come over the ramp
         * Shoot on the Alliance side
         * Head back through the trench Come back over the ramp and line up to shoot again, using the fuel collected in the first cycle.
         */ 

                public AutoRoutine RtTrench_Ramp_Sweep_Purge() {

                final AutoRoutine routine = m_factory.newRoutine("Right Trench-Ramp-Sweep_Purge");

                // Cycle 1
                // Run to the middle and while extened the intake, collect, and line up to cross the ramp
                final AutoTrajectory RtTrench_Middle = routine.trajectory("RtTrench_Middle", 0);

                // Cross the ramp to the Alliance side, and shoot with vision alignment
                final AutoTrajectory RtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
                
                // Shoot the fuel (command below)

                // Return back to the trench
                final AutoTrajectory RtShootRamp_Trench = routine.trajectory("RtShootRamp_Trench", 0);

                // Cycle 2: Fresh instances; reusing the same AutoTrajectory object

                // Sweep and collect fuel behind the hub
                final AutoTrajectory RtTrench_HubSweep = routine.trajectory("RtTrench_HubSweep", 0);

                // Turn and purge fuel over the ramp
                final AutoTrajectory RtHub_Purge = routine.trajectory("RtHub_Purge", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Middle.resetOdometry(),

                                                // --- Cycle 1 ---
                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                RtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing to Alliance side
                                                RtRampMiddle_Alliance.cmd(),

                                                // 4. Shoot — starts only after RtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout),

                                                // --- Cycle 2 ---
                                                // 5. Back to trench
                                                RtShootRamp_Trench.cmd(),

                                                // 6. Trench to Middle (2nd) — intake runs in parallel
                                                Commands.deadline(
                                                                RtTrench_HubSweep.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),
                                                RtHub_Purge.cmd(),

                                                FuelCommands.purgeFuel(m_intake, m_indexer).withTimeout(purgeTimeout) // purge command; adjust timeout as needed

                                ));

                return routine;
        }

        // FIXME
        public AutoRoutine RtTrench_Ramp_Sweep_AngryPurge() {

                final AutoRoutine routine = m_factory.newRoutine("Right Trench-Ramp-Sweep_AngryPurge");

                // Cycle 1
                final AutoTrajectory RtTrench_Middle = routine.trajectory("RtTrench_Middle", 0); // + Intake
                final AutoTrajectory RtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
                // Shoot
                final AutoTrajectory RtShootRamp_Trench = routine.trajectory("RtShootRamp_Trench", 0);

                // Cycle 2 (fresh instances; reusing the same AutoTrajectory object
                // causes its event triggers to not re-fire on the second activation)
                final AutoTrajectory RtTrench_HubSweep = routine.trajectory("RtTrench_HubSweep", 0);
                final AutoTrajectory RtHub_Purge = routine.trajectory("RtHub_Purge", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                RtTrench_Middle.resetOdometry(),

                                                // --- Cycle 1 ---
                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                RtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing to Alliance side
                                                RtRampMiddle_Alliance.cmd(),

                                                // 4. Shoot — starts only after RtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout),

                                                // --- Cycle 2 ---
                                                // 5. Back to trench
                                                RtShootRamp_Trench.cmd(),

                                                // 6. Trench to Middle (2nd) — intake runs in parallel
                                                Commands.deadline(
                                                                RtTrench_HubSweep.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                RtHub_Purge.cmd(),

                                                FuelCommands.purgeFuel(m_intake, m_indexer).withTimeout(purgeTimeout) // purge command; adjust timeout as needed

                                ));

                return routine;
        }

        // =======================================================================
        // LEFT SIDE AUTOS - start in left trench
        // =======================================================================

                // Left Trench to Middle to Ramp Shot
        public AutoRoutine LtTrench_Ramp_Single() {

                final AutoRoutine routine = m_factory.newRoutine("Left x1 Trench-Ramp");

                // Trajectories
                final AutoTrajectory LtTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
                final AutoTrajectory LtRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);
                final AutoTrajectory LtShootRamp = routine.trajectory("LtShootRamp", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTrench_Middle.resetOdometry(),

                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                LtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing
                                                LtRampMiddle_Alliance.cmd(),

                                                // 3. Drive to shoot position
                                                LtShootRamp.cmd(),

                                                // 4. Shoot — starts only after LtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout)
                                ));

                return routine;
        }

        // Left Trench to Middle to Ramp Shot
        public AutoRoutine LtTrench_Ramp_Double() {

                final AutoRoutine routine = m_factory.newRoutine("Left x2 Trench-Ramp");

                // Trajectories — cycle 1
                final AutoTrajectory LtTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
                final AutoTrajectory LtRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);

                // 2nd segment 
                final AutoTrajectory LtShootRamp_Trench = routine.trajectory("LtShootRamp_Trench", 0);

                // Trajectories — cycle 2 (fresh instances; reusing the same AutoTrajectory object
                // causes its event triggers to not re-fire on the second activation)
                final AutoTrajectory LtTrench_Middle_2 = routine.trajectory("LtTrench_Middle", 0);
                final AutoTrajectory LtRampMiddle_Alliance_2 = routine.trajectory("LtRampMiddle_Alliance", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTrench_Middle.resetOdometry(),

                                                // --- Cycle 1 ---
                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                LtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing
                                                LtRampMiddle_Alliance.cmd(),

                                                // 3. Drive from ShootRamp position back to trench for cycle 2; placeholder trajectory for now
                                                LtShootRamp_Trench.cmd(),

                                                // 4. Shoot — starts only after LtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout),

                                                // --- Cycle 2 ---
                                                // 5. Back to trench
                                                LtShootRamp_Trench.cmd(),

                                                // 6. Trench to Middle (2nd) — intake runs in parallel
                                                Commands.deadline(
                                                                LtTrench_Middle_2.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 7. Ramp crossing to Alliance side (2nd)
                                                LtRampMiddle_Alliance_2.cmd(),

                                                // 8. Shoot (2nd) — starts only after LtShootRamp_2 fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout)
                                ));

                return routine;
        }

                // Left Trench to Middle to Ramp Shot
        public AutoRoutine LtTrench_Ramp_HubSweep() {

                final AutoRoutine routine = m_factory.newRoutine("Left Trench-Ramp-HubSweep");

                // Trajectories — cycle 1
                final AutoTrajectory LtTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
                final AutoTrajectory LtRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);

                // 2nd segment 
                final AutoTrajectory LtShootRamp_Trench = routine.trajectory("LtShootRamp_Trench", 0);

                // Trajectories — cycle 2 (fresh instances; reusing the same AutoTrajectory object
                // causes its event triggers to not re-fire on the second activation)
                final AutoTrajectory LtTrench_HubSweep = routine.trajectory("LtTrench_HubSweep", 0);
                final AutoTrajectory LtHub_Ramp = routine.trajectory("LtHub_Ramp", 0);
                final AutoTrajectory LtRampMiddle_Alliance_2 = routine.trajectory("LtRampMiddle_Alliance", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTrench_Middle.resetOdometry(),

                                                // --- Cycle 1 ---
                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                LtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing
                                                LtRampMiddle_Alliance.cmd(),

                                                // 3. Drive from ShootRamp position back to trench for cycle 2; placeholder trajectory for now
                                                LtShootRamp_Trench.cmd(),

                                                // 4. Shoot — starts only after LtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout),

                                        
                                                // 5. Back to trench
                                                LtShootRamp_Trench.cmd(),

                                                // --- Cycle 2 ---
                                                // 6. Trench to Middle (2nd) — intake runs in parallel
                                                Commands.deadline(
                                                                LtTrench_HubSweep.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),
                                                LtHub_Ramp.cmd(),

                                                // 7. Ramp crossing to Alliance side (2nd)
                                                LtRampMiddle_Alliance_2.cmd(),

                                                // 8. Shoot (2nd) — starts only after LtShootRamp_2 fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout)
                                ));

                return routine;
        }

                public AutoRoutine LtTrench_Ramp_Sweep_Purge() {

                final AutoRoutine routine = m_factory.newRoutine("Left Trench-Ramp-Sweep");

                // Trajectories — cycle 1
                final AutoTrajectory LtTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
                final AutoTrajectory LtRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);

                // 2nd segment 
                final AutoTrajectory LtShootRamp_Trench = routine.trajectory("LtShootRamp_Trench", 0);

                // Trajectories — cycle 2 (fresh instances; reusing the same AutoTrajectory object
                // causes its event triggers to not re-fire on the second activation)
                final AutoTrajectory LtTrench_HubSweep = routine.trajectory("LtTrench_HubSweep", 0);
                final AutoTrajectory LtHub_Purge = routine.trajectory("LtHub_Purge", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTrench_Middle.resetOdometry(),

                                                // --- Cycle 1 ---
                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                LtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing
                                                LtRampMiddle_Alliance.cmd(),

                                                // 3. Drive from ShootRamp position back to trench for cycle 2; placeholder trajectory for now
                                                LtShootRamp_Trench.cmd(),

                                                // 4. Shoot — starts only after LtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout),

                                        
                                                // 5. Back to trench
                                                LtShootRamp_Trench.cmd(),

                                                // --- Cycle 2 ---
                                                // 6. Trench to Middle (2nd) — intake runs in parallel
                                                Commands.deadline(
                                                                LtTrench_HubSweep.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),
                                                LtHub_Purge.cmd(),
                                                
                                                FuelCommands.purgeFuel(m_intake, m_indexer).withTimeout(purgeTimeout) // purge command; adjust timeout as needed
                                ));

                return routine;
        }

         public AutoRoutine LtTrench_Ramp_Sweep_AngryPurge() {

                final AutoRoutine routine = m_factory.newRoutine("Left Trench-Ramp-Sweep");

                // Trajectories — cycle 1
                final AutoTrajectory LtTrench_Middle = routine.trajectory("LtTrench_Middle", 0);
                final AutoTrajectory LtRampMiddle_Alliance = routine.trajectory("LtRampMiddle_Alliance", 0);

                // 2nd segment 
                final AutoTrajectory LtShootRamp_Trench = routine.trajectory("LtShootRamp_Trench", 0);

                // Trajectories — cycle 2 (fresh instances; reusing the same AutoTrajectory object
                // causes its event triggers to not re-fire on the second activation)
                final AutoTrajectory LtTrench_HubSweep = routine.trajectory("LtTrench_HubSweep", 0);
                final AutoTrajectory LtHub_Purge = routine.trajectory("LtHub_Purge", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTrench_Middle.resetOdometry(),

                                                // --- Cycle 1 ---
                                                // 1. Trench to Middle — intake runs in parallel while driving
                                                Commands.deadline(
                                                                LtTrench_Middle.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),

                                                // 2. Ramp crossing
                                                LtRampMiddle_Alliance.cmd(),

                                                // 3. Drive from ShootRamp position back to trench for cycle 2; placeholder trajectory for now
                                                LtShootRamp_Trench.cmd(),

                                                // 4. Shoot — starts only after LtShootRamp fully completes
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout),

                                        
                                                // 5. Back to trench
                                                LtShootRamp_Trench.cmd(),

                                                // --- Cycle 2 ---
                                                // 6. Trench to Middle (2nd) — intake runs in parallel
                                                Commands.deadline(
                                                                LtTrench_HubSweep.cmd(),
                                                                m_intake.intakeFuelTimer(intakeTimeout, intakeDelay)),
                                                LtHub_Purge.cmd(),
                                                
                                                FuelCommands.purgeFuel(m_intake, m_indexer).withTimeout(purgeTimeout) // purge command; adjust timeout as needed
                                ));

                return routine;
        }

        // ============================================================================
        // Secondary Auto Routines
        // ============================================================================
        
 public AutoRoutine Bulldozer() {
                final AutoRoutine routine = m_factory.newRoutine("Bulldozer 2026");

                final AutoTrajectory Bulldozer = routine.trajectory("Bulldozer2026", 0);
                final AutoTrajectory RtRampMiddle_Alliance = routine.trajectory("RtRampMiddle_Alliance", 0);
                
                routine.active().onTrue(
                                Commands.sequence(
                                                Bulldozer.resetOdometry(),
                                                
                                                Bulldozer.cmd(),
                                                
                                                RtRampMiddle_Alliance.cmd(),
                                                
                                                FuelCommands.Auto.poseAlignAndShoot(m_shooter, m_indexer, m_intake, m_drivetrain, shootTimeout)
                                                

                                ));
                // Routine Events
                Bulldozer.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout + 2, intakeDelay));

                return routine;
        }


        


        // ============================================================================
        // Experimental
        // ============================================================================

        // Inner class for testing new auto features without affecting existing routines
        public class Test {
        

        public AutoRoutine Full_LtTrench_Mid_Trench() {
                final AutoRoutine routine = m_factory.newRoutine("FULL Lt Trench-Mid-Trench"); 
                final AutoTrajectory LtTrench_Mid_Trench = routine.trajectory("Full_LtTrench_Mid_Trench", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                LtTrench_Mid_Trench.resetOdometry(), // Always reset odometry first
                                                LtTrench_Mid_Trench.cmd() // Follow the path

                                ));
                // Routine Events
                LtTrench_Mid_Trench.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout, intakeDelay));
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
                MidDepot.atTime("Intake").onTrue(m_intake.intakeFuelTimer(8, intakeDelay));
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
                                .onTrue(FuelCommands.Auto.shootClose(m_shooter, m_indexer,shootTimeout));
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
                RtTrench_RtMid_RtTrench.atTime("Intake").onTrue(m_intake.intakeFuelTimer(intakeTimeout, intakeDelay));
                
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