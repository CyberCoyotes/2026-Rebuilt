package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotPreset;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Fuel Commands - Factory for shooter-related commands.
 *
 * This class provides static factory methods to create common shooter commands.
 * Using a factory pattern keeps command creation centralized and reusable.
 * 
 */
public class FuelCommands {

    // =========================================================================
    // PRIMARY SHOOT COMMANDS
    // =========================================================================

    /**
     * Shoots at whatever preset is currently selected.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @return Command that shoots at current targets and returns to idle on release
     */

    public static Command shootAtCurrentTarget(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.runOnce(shooter::prepareToShoot, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.indexerForward();
                    indexer.conveyorForward();
                }, indexer)).finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle(); // Stop all flywheel motors on trigger release
                }).withName("ShootAtCurrentTarget");
    }

    /**
     * Full preset shoot sequence that owns both the shooter and indexer subsystems.
     *
     * Flow:
     * 1. Sets the given RPM + hood target silently
     * 2. Calls prepareToShoot() — flywheel ramps up, hood moves to target
     * 3. Waits until both are at target (isReady()), with a 3-second safety timeout
     * 4. Runs indexer and conveyor forward to feed the game piece
     * 5. On trigger release (whileTrue interrupt): stops indexer/conveyor, returns
     * shooter to standby
     *
     * Use with whileTrue() on the shoot trigger.
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param rpm     Target flywheel velocity in RPM
     * @param hood    Target hood position in rotations
     * @return Complete preset shoot command requiring both subsystems
     */
    public static Command shootWithPreset(ShooterSubsystem shooter, IndexerSubsystem indexer,
            double rpm, double hood) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(rpm); // Set Constants._RPM
                    shooter.setTargetHoodPose(hood); // Set Constants._HOOD
                    shooter.prepareToShoot();
                }, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.feed();
                }, indexer))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                }).withName("ShootWithPreset[" + rpm + "rpm]");
    }

    /**
     * Shoots at the given RPM/hood preset while simultaneously retracting the
     * intake slides (two-phase) and running the intake roller.
     *
     * The shooter+indexer sequence is the deadline — slide retraction runs in
     * parallel and is cancelled when the shot completes or the trigger is released.
     * The intake roller is always stopped on exit via the intake command's finallyDo.
     *
     * Use wherever {@link #shootWithPreset} is used when slides need to retract
     * during the shot (e.g. after intaking from the trench).
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @param intake  The intake subsystem
     * @param rpm     Target flywheel velocity in RPM
     * @param hood    Target hood position in rotations
     * @return Deadline command: shoot sequence (deadline) + slide retract in parallel
     */
    public static Command shootWithSlideRetract(ShooterSubsystem shooter, IndexerSubsystem indexer,
            IntakeSubsystem intake, double rpm, double hood) {
        return Commands.deadline(
                shootWithPreset(shooter, indexer, rpm, hood),
                intake.retractSlidesWithRollerCmd())
                .withName("ShootWithSlideRetract[" + rpm + "rpm]");
    }

    // Template for the using ShotPresets in Auton
    public static Command shootPresetAuton(ShooterSubsystem shooter, IndexerSubsystem indexer,
            ShotPreset preset, double feedSeconds,
            boolean waitForReady) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(preset.rpm);
                    shooter.setTargetHoodPose(preset.hood);
                    shooter.prepareToShoot();
                }, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.indexerForward();
                    indexer.conveyorForward();
                }, indexer).withTimeout(feedSeconds) // primary end trigger: timeout
        ).finallyDo(() -> {
            indexer.indexerStop();
            indexer.conveyorStop();
            shooter.setIdle();
        }).withName("ShootPresetAuton[" + preset.label + "]");
    }

    /**
     * Full shoot sequence using the preset selected via button input
     * Defaults to the CLOSE preset (Close RPM + Close hood) on robot startup.
     *
     * Flow:
     * Button selects a RPM & hood preset silently
     * Shooter trigger behavior:
     * Calls `prepareToShoot()` — flywheel ramps up, hood moves to target
     * Waits until both are at target (isReady()), with a 3-second safety timeout
     * Calls `feed()` - runs indexer and conveyor forward to move game piece toward
     * the flywheel
     * On trigger release (whileTrue interrupt): stops indexer/conveyor, returns
     * shooter
     *
     * @param shooter The shooter subsystem
     * @param indexer The indexer subsystem
     * @return Complete preset shoot command using the currently selected preset
     */
    public static Command shootWithSelectedPreset(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setSelectedPreset();
                    shooter.prepareToShoot();
                }, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.indexerForward();
                    indexer.conveyorForward();
                }, indexer)).finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle(); // Stop all flywheel motors on trigger release
                }).withName("ShootWithSelectedPreset");
    }

    // =========================================================================
    // SILENT PRESET COMMANDS
    // =========================================================================
    // These update the target RPM and hood angle, silently without calling
    // prepareToShoot() or changing the shooter's state.

    /**
     * Silently selects close shot targets (CLOSE_SHOT_RPM + CLOSE_SHOT_HOOD).
     *
     * @param shooter The shooter subsystem
     * @return Command that silently sets close shot targets
     */
    public static Command setCloseShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setCloseShotPreset, shooter)
                .withName("SetCloseShot");
    }

    /**
     * Silently arms far shot targets (FAR_SHOT_RPM + FAR_SHOT_HOOD).
     *
     * @param shooter The shooter subsystem
     * @return Command that silently sets far shot targets
     */
    public static Command setFarShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setFarShotPreset, shooter)
                .withName("SetFarShot");
    }

    /**
     * Silently se pass shot targets (PASS_SHOT_RPM + PASS_SHOT_HOOD).
     *
     * @param shooter The shooter subsystem
     * @return Command that silently sets pass shot targets
     */
    public static Command setPassShot(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::setPassShotPreset, shooter)
                .withName("SetPassShot");
    }

    /**
     * Air popper assist _without_ intake.
     *
     * Holds the shooter at POPPER preset while feeding the indexer/conveyor.
     * Use when an intake command is already being scheduled elsewhere.
     */
    // public static Command runAirPopperTest(IndexerSubsystem indexer,
    // ShooterSubsystem shooter) {
    // return Commands.sequence(
    // Commands.runOnce(() -> {
    // shooter.setAirPopper();
    // shooter.prepareToShoot();
    // }, shooter),
    // Commands.deadline(
    // indexer.feed(),
    // Commands.run(() -> {
    // }, shooter))).finallyDo(() -> {
    // indexer.indexerStop();
    // indexer.conveyorStop();
    // shooter.setIdle();
    // }).withName("RunAirPopper");
    // }

    /**
     * Air popper assist with intake.
     *
     * While held: runs intake + indexer feed while shooter stays in POPPER preset.
     */
    public static Command runAirPopper(IndexerSubsystem indexer, ShooterSubsystem shooter,
            IntakeSubsystem intake) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setAirPopper();
                    shooter.prepareToShoot();
                }, shooter),
                Commands.deadline(
                        Commands.deadline( // parallel is NOT OK here!
                                intake.intakeFuel(),
                                indexer.feed()),
                        Commands.run(() -> {
                        }, shooter)))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                }).withName("RunAirPopperWithIntake");
    }

    // =========================================================================
    // UTILITY COMMANDS
    // =========================================================================

    /**
     * Creates a command to eject jammed game pieces.
     * Reverses flywheel for a duration then returns to SPINUP.
     *
     * @param shooter         The shooter subsystem
     * @param durationSeconds How long to eject
     * @return Command that ejects then returns to idle
     */
    public static Command eject(ShooterSubsystem shooter, double durationSeconds) {
        return Commands.sequence(
                Commands.runOnce(shooter::eject, shooter),
                Commands.waitSeconds(durationSeconds)
        // Commands.runOnce(shooter::returnToStandby, shooter) // TODO: Do not use right
        // now **EXPERIMENTAL**
        ).withName("EjectShooter");
    }

    /**
     * Creates a command that waits until shooter is ready.
     *
     * @param shooter The shooter subsystem
     * @return Command that waits for shooter ready state
     */
    public static Command waitUntilReady(ShooterSubsystem shooter) {
        return Commands.waitUntil(shooter::isReady)
                .withTimeout(3.0)
                .withName("WaitForShooterReady");
    }

    public static Command shootPass(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(Constants.Shooter.PASS_RPM);
                    shooter.setTargetHoodPose(Constants.Shooter.PASS_HOOD);
                    shooter.prepareToShoot();
                }, shooter),
                Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                Commands.run(() -> {
                    indexer.feed();
                }, indexer))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                }).withName("ShootPass");
    }

    /**
     * Creates a command to return shooter to SPINUP at STANDBY_RPM.
     *
     * @param shooter The shooter subsystem
     * @return Command that returns shooter to standby spinning state
     */

    /** TODO: Do not use right now _EXPERIMENTAL_ */
    // public static Command returnToStandby(ShooterSubsystem shooter) {
    // return Commands.runOnce(shooter::returnToStandby, shooter)
    // .withName("ReturnToStandby");
    // }

    // =========================================================================
    // VISION ALIGN AND SHOOT (primary match command)
    // =========================================================================

   

    public static class Auto {

        /* Autonomous shooting command */
        /* FIXME Trench is the working "Test" command for autonomous. Others should be updated after its working properly */
        public static Command shootTrench(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.TRENCH_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.TRENCH_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady), // removed timeout
                    indexer.feed().withTimeout(feedSeconds) // removed sensor time
            ).finallyDo(() -> {
                indexer.indexerStop();
                indexer.conveyorStop();
                shooter.setIdle();
            }).withName("ShootTrenchAuton");
        }

        /**
         * Autonomous: shoot at trench preset while retracting intake slides (two-phase)
         * and running the intake roller concurrently.
         *
         * The shoot sequence ({@link #shootTrench}) is the deadline — slide retraction
         * is cancelled when feeding completes. Safe to compose with path-following or
         * other parallel commands in auton.
         *
         * @param shooter     The shooter subsystem
         * @param indexer     The indexer subsystem
         * @param intake      The intake subsystem
         * @param feedSeconds How long to run the indexer/conveyor after ready
         * @return Deadline command: shootTrench (deadline) + slide retract in parallel
         */
        public static Command shootTrenchWithSlideRetract(ShooterSubsystem shooter,
                IndexerSubsystem indexer, IntakeSubsystem intake, double feedSeconds) {
            return Commands.deadline(
                    shootTrench(shooter, indexer, feedSeconds),
                    intake.retractSlidesWithRollerCmd())
                    .withName("ShootTrenchWithSlideRetract");
        }

        /* Autonomous shooting command */
        public static Command shootHub(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.CLOSE_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.CLOSE_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady), // remove timeout

                    // remove indexer::donePassingFuel for now
                    indexer.feed().withTimeout(feedSeconds)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootHubAuton");
        }

        /* Autonomous shooting command */
        public static Command shootTower(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.TOWER_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.TOWER_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feed().until(indexer::donePassingFuel).withTimeout(feedSeconds)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootTowerAuton");
        } // end of command
        /* Autonomous shooting command */

        public static Command shootFar(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.FAR_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.FAR_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feed().until(indexer::donePassingFuel).withTimeout(feedSeconds)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootFarAuton");
        } // end of command

        /* Autonomous shooting command */
        public static Command shootTren(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(Constants.Shooter.TRENCH_RPM);
                        shooter.setTargetHoodPose(Constants.Shooter.TRENCH_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(6.0),
                    indexer.feed().until(indexer::donePassingFuel).withTimeout(feedSeconds)).finallyDo(() -> {
                        indexer.indexerStop();
                        indexer.conveyorStop();
                        shooter.setIdle();
                    }).withName("ShootTrenchAuton");
        }
    } // end of class Auto

} // end of class FuelCommands