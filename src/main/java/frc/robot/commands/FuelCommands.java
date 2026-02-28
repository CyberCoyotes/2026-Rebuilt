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
                Commands.waitUntil(shooter::isReady).withTimeout(1.0),
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
    // These update the target RPM and hood angle, silently without calling prepareToShoot() or changing the shooter's state.

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

    public static Command shootPass(
            ShooterSubsystem shooter, IndexerSubsystem indexer, double rpm, double hood) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.setTargetVelocity(ShooterSubsystem.PASS_RPM);
                    shooter.setTargetHoodPose(ShooterSubsystem.PASS_HOOD);
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

    /**
     * Combined vision-targeting and shooting command. Intended as the primary RT
     * binding.
     *
     * Behavior (all while trigger held):
     * 1. Arms the current POV-selected preset immediately as a fallback baseline.
     * 2. Continuously looks for the nearest hub AprilTag (alliance-aware).
     * 3. If a hub tag is visible (or in grace period): updates flywheel RPM and
     * hood position from the distance interpolation table every cycle.
     * 4. Overrides drivetrain rotation with a P-controller on tx (horizontal
     * angle),
     * while still allowing the driver to translate freely with the left stick.
     * 5. Fires the indexer and conveyor as soon as vision reports aligned AND
     * shooter reports ready. Stops feeding if either condition is lost.
     * 6. On trigger release: stops indexer, conveyor, and shooter.
     *
     * Subsystem requirements: shooter, vision, indexer, drivetrain
     * → interrupts the default drive command for the duration of the trigger hold.
     *
     * Tuning handles:
     * - Constants.Vision.ROTATIONAL_KP (rotation aggressiveness)
     * - Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC (rotation clamp)
     * - ShooterSubsystem FLYWHEEL_RPM_MAP / HOOD_ROT_MAP (shooter params)
     * See TUNING.md for the step-by-step procedure.
     *
     * @param shooter    Shooter subsystem
     * @param vision     Vision subsystem
     * @param indexer    Indexer subsystem
     * @param drivetrain Swerve drivetrain
     * @param xSupplier  Driver left-Y velocity in m/s (already scaled by MaxSpeed)
     * @param ySupplier  Driver left-X velocity in m/s (already scaled by MaxSpeed)
     */
    public static Command visionAlignAndShoot(
            ShooterSubsystem shooter,
            VisionSubsystem vision,
            IndexerSubsystem indexer,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {

        final double maxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        // Created once; reused every execute cycle.
        // FieldCentric: driver controls X/Y translation, vision controls rotation.
        final SwerveRequest.FieldCentric alignRequest = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.15)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return Commands.run(() -> {

            // ── 1. Shooter: update from vision or hold current targets ─────────
            if (vision.isUsableForShooting()) {
                // Live-update RPM + hood from distance table (also pushes to hardware
                // if already in READY state — see ShooterSubsystem.updateFromDistance)
                shooter.updateFromDistance(vision.getDistanceToTargetMeters());
            }
            // Keep commanding READY every cycle — setState() no-ops if already there
            if (shooter.getState() != ShooterSubsystem.ShooterState.READY) {
                shooter.prepareToShoot();
            }

            // ── 2. Drivetrain: driver translation + vision rotation ────────────
            // tx returns 0.0 when NO_TARGET, so rotation correction drops to zero
            // automatically when the camera has nothing to track.
            double txDeg = vision.getHorizontalAngleDegrees();
            double rotRate = MathUtil.clamp(
                    txDeg * Constants.Vision.ROTATIONAL_KP,
                    -Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC,
                    Constants.Vision.MAX_ALIGNMENT_ROTATION_RAD_PER_SEC);

            drivetrain.setControl(
                    alignRequest
                            .withVelocityX(xSupplier.getAsDouble())
                            .withVelocityY(ySupplier.getAsDouble())
                            .withRotationalRate(rotRate));

            // ── 3. Feed: only when both conditions met ─────────────────────────
            if (vision.isAligned() && shooter.isReady()) {
                indexer.indexerForward();
                indexer.conveyorForward();
            } else {
                indexer.indexerStop();
                indexer.conveyorStop();
            }

        }, shooter, vision, indexer, drivetrain)
                .beforeStarting(Commands.runOnce(() -> {
                    shooter.setSelectedPreset(); // Spin up to the POV-selected preset immediately
                    shooter.prepareToShoot(); // Enter READY state — don't wait for alignment
                }, shooter))
                .finallyDo(() -> {
                    indexer.indexerStop();
                    indexer.conveyorStop();
                    shooter.setIdle();
                })
                .withName("VisionAlignAndShoot");
    }

    /**
     * Creates a vision-based shoot sequence with indexer coordination.
     *
     * @param shooter The shooter subsystem
     * @param vision  The vision subsystem
     * @param indexer The indexer subsystem
     * @return Vision-based shooting sequence
     */
    public static Command visionShootSequence(ShooterSubsystem shooter, VisionSubsystem vision,
            IndexerSubsystem indexer) {
        return Commands.sequence(
                Commands.waitUntil(vision::hasTarget).withTimeout(1.0),
                visionShot(shooter, vision)
        // IndexerCommands.feedTimed(indexer, 0.5), // FIXME to use the
        // IndexerSubsystem's feed method instead of a command from IndexerCommands
        // Commands.runOnce(shooter::returnToStandby, shooter)
        ).withName("VisionShootSequence");
    }

    /**
     * Creates a command to shoot using vision-based distance calculation.
     *
     * @param shooter The shooter subsystem
     * @param vision  The vision subsystem
     * @return Command that uses vision for aiming
     */
    public static Command visionShot(ShooterSubsystem shooter, VisionSubsystem vision) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    double distance = vision.getDistanceToTargetMeters();
                    shooter.updateFromDistance(distance);
                    shooter.prepareToShoot();
                }, shooter, vision),
                Commands.waitUntil(shooter::isReady)).withTimeout(3.0)
                .withName("VisionShot");
    }

    /**
     * Creates a command to continuously update shooter based on vision.
     *
     * @param shooter The shooter subsystem
     * @param vision  The vision subsystem
     * @return Command that continuously tracks target
     */

    public static Command trackTarget(ShooterSubsystem shooter, VisionSubsystem vision) {
        return Commands.run(() -> {
            if (vision.hasTarget()) {
                double distance = vision.getDistanceToTargetMeters();
                shooter.updateFromDistance(distance);
            }
        }, shooter, vision)
                .withName("TrackTarget");
    }

    // ========================================================================
    // AUTONOMOUS SHOOTING COMMANDS
    // ========================================================================
    public static class Auto {
        /**
         * Autonomous: shoot using the TRENCH preset.
         * Less verbose for other auton shooting presets*
         * 
         * @param shooter     The shooter subsystem
         * @param indexer     The indexer subsystem
         * @param feedSeconds How long to run the indexer/conveyor after ready
         * @return Autonomous TRENCH shoot command
         */
        public static Command shootTrench(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(ShooterSubsystem.TRENCH_RPM);
                        shooter.setTargetHoodPose(ShooterSubsystem.TRENCH_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady), // .withTimeout(3.0),

                    // Feed until chute goes quiet for 2s (with safety timeout)
                    indexer.feed().until(indexer::donePassingFuel)
            /*
             * This is likely redundant with the feed().until() end condition, but it's a
             * good safety net in case of sensor issues or unexpected game piece behavior.
             * .finallyDo(() -> {
             * indexer.indexerStop();
             * indexer.conveyorStop();
             * shooter.setIdle();
             * })
             */
            ).withName("ShootTrenchAuton");

        } // end of command shooting from the Trench position

        public static Command shootHub(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(ShooterSubsystem.CLOSE_RPM);
                        shooter.setTargetHoodPose(ShooterSubsystem.CLOSE_HOOD);
                        shooter.prepareToShoot();
                    }, shooter),
                    Commands.waitUntil(shooter::isReady).withTimeout(3.0),
                    Commands.run(() -> {
                        indexer.feed();
                    }, indexer).withTimeout(feedSeconds) // primary end trigger: timeout
            ).finallyDo(() -> {
                indexer.indexerStop();
                indexer.conveyorStop();
                shooter.setIdle();
            });

        } // end of command

        public static Command shootTower(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(ShooterSubsystem.TOWER_RPM);
                        shooter.setTargetHoodPose(ShooterSubsystem.TOWER_HOOD);
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
            });
        } // end of command

        public static Command shootCorner(ShooterSubsystem shooter, IndexerSubsystem indexer,
                double feedSeconds) {
            return Commands.sequence(
                    Commands.runOnce(() -> {
                        shooter.setTargetVelocity(ShooterSubsystem.FAR_RPM);
                        shooter.setTargetHoodPose(ShooterSubsystem.FAR_HOOD);
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
            });
        } // end of command

    } // end of class Auto

} // end of class FuelCommands