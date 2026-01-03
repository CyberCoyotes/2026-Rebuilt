package main.java.frc.robot.training.scoy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.training.LightsaberHilt.State;

/**
 * LightsaberHilt Commands - STATE MACHINE COMMAND EXAMPLE
 * 
 * This example demonstrates more advanced command patterns:
 * - Commands that REQUEST states and WAIT for completion
 * - Sequenced commands (do A, then B, then C)
 * - Conditional commands with state checks
 * - Timeout handling for safety
 * 
 * Key concept: Commands don't directly control motors here!
 * They request states from the subsystem's state machine.
 * The state machine (in periodic()) handles the actual motor control.
 * 
 * This separation keeps logic clean:
 * - Commands = "what to do" (high-level intent)
 * - State machine = "how to do it" (low-level execution)
 */
public class LightsaberHiltCommands {
    
    private LightsaberHiltCommands() {}
    
    // ========== BASIC STATE COMMANDS ==========
    
    /**
     * Ignite the lightsaber and wait until fully extended.
     * 
     * This command:
     * 1. Requests ignite
     * 2. Waits until blade is fully extended
     * 3. Completes (allowing next command in a sequence)
     * 
     * @param saber the LightsaberHilt subsystem
     * @return command that ignites and waits for ready
     */
    public static Command ignite(LightsaberHilt saber) {
        return Commands.sequence(
            // Step 1: Request the ignite
            Commands.runOnce(() -> saber.requestIgnite(), saber),
            // Step 2: Wait until fully extended (or timeout after 2 seconds)
            Commands.waitUntil(() -> saber.isReadyForCombat())
                .withTimeout(2.0)
        );
    }
    
    /**
     * Extinguish the lightsaber and wait until fully retracted.
     * 
     * @param saber the LightsaberHilt subsystem
     * @return command that extinguishes and waits for safe
     */
    public static Command extinguish(LightsaberHilt saber) {
        return Commands.sequence(
            Commands.runOnce(() -> saber.requestExtinguish(), saber),
            Commands.waitUntil(() -> saber.isSafeToHolster())
                .withTimeout(2.0)
        );
    }
    
    /**
     * Hold ignite while button pressed, extinguish on release.
     * Good for "momentary" lightsaber activation.
     * 
     * Note: Uses startEnd pattern - start requests ignite, end requests extinguish.
     * The state machine handles the actual extension/retraction.
     * 
     * @param saber the LightsaberHilt subsystem
     * @return command that keeps saber ignited while held
     */
    public static Command holdIgnite(LightsaberHilt saber) {
        return Commands.startEnd(
            () -> saber.requestIgnite(),
            () -> saber.requestExtinguish(),
            saber
        );
    }
    
    // ========== SMART/CONDITIONAL COMMANDS ==========
    
    /**
     * Toggle the lightsaber - if retracted, ignite; if extended, extinguish.
     * 
     * @param saber the LightsaberHilt subsystem
     * @return command that toggles saber state
     */
    public static Command toggle(LightsaberHilt saber) {
        return Commands.either(
            // If currently retracted: ignite
            ignite(saber),
            // If not retracted: extinguish
            extinguish(saber),
            // Condition: check if safe to holster (meaning currently retracted)
            () -> saber.isSafeToHolster()
        );
    }
    
    /**
     * Only ignite if currently retracted. Does nothing otherwise.
     * Safer than raw ignite() for button bindings.
     * 
     * @param saber the LightsaberHilt subsystem
     * @return command that ignites only from retracted state
     */
    public static Command igniteIfSafe(LightsaberHilt saber) {
        return Commands.either(
            ignite(saber),
            Commands.none(),
            () -> saber.isSafeToHolster()
        );
    }
    
    // ========== COMBAT SEQUENCE EXAMPLE ==========
    
    /**
     * Execute a "flourish" - ignite, hold for a moment, extinguish.
     * Demonstrates command sequencing for complex behaviors.
     * 
     * @param saber the LightsaberHilt subsystem
     * @return command sequence for lightsaber flourish
     */
    public static Command flourish(LightsaberHilt saber) {
        return Commands.sequence(
            // Step 1: Ignite
            ignite(saber),
            // Step 2: Hold extended for 1 second
            Commands.waitSeconds(1.0),
            // Step 3: Extinguish
            extinguish(saber)
        );
    }
    
    /**
     * Combat ready sequence - ignite and stay ready.
     * Prints status to show what's happening.
     * 
     * @param saber the LightsaberHilt subsystem
     * @return command that prepares for combat
     */
    public static Command prepareForCombat(LightsaberHilt saber) {
        return Commands.sequence(
            // Print intent
            Commands.print("Preparing lightsaber for combat..."),
            // Check if already ready
            Commands.either(
                // Already ready - just print
                Commands.print("Lightsaber already ignited!"),
                // Not ready - ignite
                ignite(saber),
                () -> saber.isReadyForCombat()
            ),
            // Confirm ready
            Commands.print("Ready for combat!")
        );
    }
    
    // ========== STATE MONITORING COMMAND ==========
    
    /**
     * Run continuously, printing state changes.
     * Useful for debugging state machine behavior.
     * 
     * @param saber the LightsaberHilt subsystem
     * @return command that monitors and logs state
     */
    public static Command monitorState(LightsaberHilt saber) {
        // Track previous state to detect changes
        final State[] previousState = { null };
        
        return Commands.run(() -> {
            State current = saber.getCurrentState();
            if (current != previousState[0]) {
                System.out.println("LIGHTSABER STATE: " + current.name());
                previousState[0] = current;
            }
        }, saber);
    }
    
    // ========== EXAMPLE BINDINGS ==========
    /*
     * In RobotContainer.java:
     * 
     * private void configureBindings() {
     *     // Press A to toggle lightsaber on/off
     *     driver.a().onTrue(LightsaberHiltCommands.toggle(lightsaber));
     *     
     *     // Hold right bumper for momentary ignition
     *     driver.rightBumper().whileTrue(LightsaberHiltCommands.holdIgnite(lightsaber));
     *     
     *     // Press Y for flourish (ignite, pause, extinguish)
     *     driver.y().onTrue(LightsaberHiltCommands.flourish(lightsaber));
     *     
     *     // Press B to force extinguish (emergency)
     *     driver.b().onTrue(LightsaberHiltCommands.extinguish(lightsaber));
     *     
     *     // Hold left bumper to see state in console (debugging)
     *     driver.leftBumper().whileTrue(LightsaberHiltCommands.monitorState(lightsaber));
     * }
     * 
     * // Autonomous example:
     * public Command getAutonomousCommand() {
     *     return Commands.sequence(
     *         // Dramatic ignite at start of match
     *         LightsaberHiltCommands.prepareForCombat(lightsaber),
     *         // ... rest of autonomous
     *         // Extinguish at end
     *         LightsaberHiltCommands.extinguish(lightsaber)
     *     );
     * }
     */
}
