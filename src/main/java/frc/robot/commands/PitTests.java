package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShotPreset;

/**
 * Self-contained pit test sequences that exercise the robot using the same
 * command paths used during a match.  No special test-only methods required —
 * every step goes through the standard ShooterSubsystem state machine.
 *
 * Usage (in RobotContainer or a dashboard button binding):
 *   new JoystickButton(panel, 1).onTrue(PitTests.testAll(m_shooter));
 */
public class PitTests {

    /** Seconds to wait for isReady() before giving up and moving on. */
    private static final double SPIN_UP_TIMEOUT = 5.0;

    /** Seconds to hold at each target so the operator can observe it. */
    private static final double HOLD_TIME = 2.0;

    /** Seconds to let the flywheel coast down before entering eject. */
    private static final double SPINDOWN_WAIT = 3.0;

    // Prevent instantiation — all methods are static.
    private PitTests() {}

    // =====================================================================
    // Public sequences
    // =====================================================================

    /**
     * Sequences through every ShotPreset: spins up, waits until ready (or
     * times out), holds briefly, then idles before the next preset.
     */
    public static Command testShooterPresets(ShooterSubsystem shooter) {
        Command[] steps = new Command[ShotPreset.values().length];
        ShotPreset[] presets = ShotPreset.values();
        for (int i = 0; i < presets.length; i++) {
            steps[i] = testPreset(shooter, presets[i]);
        }
        return Commands.sequence(steps).withName("PitTest: Shooter Presets");
    }

    /**
     * Exercises eject mode.  Waits for the flywheel to spin down first so the
     * velocity guard in eject() doesn't block entry.
     */
    public static Command testShooterEject(ShooterSubsystem shooter) {
        return Commands.sequence(
            Commands.waitSeconds(SPINDOWN_WAIT),
            Commands.runOnce(shooter::eject, shooter),
            Commands.waitSeconds(HOLD_TIME),
            Commands.runOnce(shooter::setIdle, shooter)
        ).withName("PitTest: Eject");
    }

    /**
     * Full pit test: all shot presets followed by an eject cycle.
     */
    public static Command testAll(ShooterSubsystem shooter) {
        return Commands.sequence(
            testShooterPresets(shooter),
            testShooterEject(shooter)
        ).withName("PitTest: All");
    }

    // =====================================================================
    // Private helpers
    // =====================================================================

    /**
     * Tests a single ShotPreset end-to-end through the normal state machine:
     *   1. Set targets (rpm + hood) and begin spinning up.
     *   2. Wait until isReady() — same gate used during a match shot.
     *   3. Hold at target so the operator can observe.
     *   4. Return to idle.
     */
    private static Command testPreset(ShooterSubsystem shooter, ShotPreset preset) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                shooter.setTargetVelocity(preset.rpm);
                shooter.setTargetHoodPose(preset.hood);
                shooter.beginSpinUp();
            }, shooter),
            Commands.waitUntil(shooter::isReady).withTimeout(SPIN_UP_TIMEOUT),
            Commands.waitSeconds(HOLD_TIME),
            Commands.runOnce(shooter::setIdle, shooter)
        ).withName("PitTest: " + preset.label);
    }
}
