package frc.robot.training;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * HyperDrive Commands - SIMPLE EXAMPLE
 * 
 * This example demonstrates basic command factory patterns:
 * - Static factory methods (no instantiation)
 * - Commands.startEnd() for "while held" actions
 * - Commands.runOnce() for instant actions
 * - Commands.run() for continuous actions
 * 
 * Commands are the "glue" between buttons and subsystem actions.
 * Think of them as reusable recipes that can be triggered by:
 * - Button presses (in RobotContainer)
 * - Autonomous routines
 * - Other commands
 */
public class HyperDriveCommands {
    
    // Private constructor - this class only has static methods
    // We never create an instance: HyperDriveCommands.charge() not new HyperDriveCommands().charge()
    private HyperDriveCommands() {}
    
    // ========== BASIC COMMANDS ==========
    
    /**
     * Charge the hyperdrive while button is held.
     * Stops charging when button is released.
     * 
     * Usage: driver.a().whileTrue(HyperDriveCommands.charge(hyperDrive));
     * 
     * @param hyperDrive the HyperDrive subsystem
     * @return command that charges while held
     */
    public static Command charge(HyperDriveSubsystem hyperDrive) {
        return Commands.startEnd(
            // Start action: begin charging
            () -> hyperDrive.charge(),
            // End action: stop (disengage returns to idle)
            () -> hyperDrive.disengage(),
            // Requires: this subsystem (prevents conflicts)
            hyperDrive
        );
    }
    
    /**
     * Engage hyperdrive while button is held.
     * Returns to realspace when button released.
     * 
     * Note: Will only work if hyperdrive isReady() returns true!
     * 
     * Usage: driver.rightTrigger().whileTrue(HyperDriveCommands.engage(hyperDrive));
     * 
     * @param hyperDrive the HyperDrive subsystem
     * @return command that engages while held
     */
    public static Command engage(HyperDriveSubsystem hyperDrive) {
        return Commands.startEnd(
            () -> hyperDrive.engage(),
            () -> hyperDrive.disengage(),
            hyperDrive
        );
    }
    
    /**
     * Emergency stop - instant action, runs once.
     * 
     * Usage: driver.b().onTrue(HyperDriveCommands.emergencyStop(hyperDrive));
     * 
     * @param hyperDrive the HyperDrive subsystem
     * @return command that immediately stops the hyperdrive
     */
    public static Command emergencyStop(HyperDriveSubsystem hyperDrive) {
        return Commands.runOnce(
            () -> hyperDrive.emergencyStop(),
            hyperDrive
        );
    }
    
    // ========== SMART COMMANDS ==========
    // These add logic to make commands "smarter"
    
    /**
     * Charge until ready, then automatically stop.
     * Use this when you want to "prep" the hyperdrive without holding a button.
     * 
     * Usage: driver.x().onTrue(HyperDriveCommands.chargeUntilReady(hyperDrive));
     * 
     * @param hyperDrive the HyperDrive subsystem
     * @return command that charges until ready threshold reached
     */
    public static Command chargeUntilReady(HyperDriveSubsystem hyperDrive) {
        return Commands.startEnd(
            () -> hyperDrive.charge(),
            () -> {}, // Don't disengage - stay at ready speed
            hyperDrive
        ).until(() -> hyperDrive.isReady());
    }
    
    /**
     * Only engage if hyperdrive is ready. Does nothing if not ready.
     * Safer than raw engage() because it won't even try if not charged.
     * 
     * Usage: driver.rightTrigger().whileTrue(HyperDriveCommands.engageIfReady(hyperDrive));
     * 
     * @param hyperDrive the HyperDrive subsystem
     * @return command that engages only when ready
     */
    public static Command engageIfReady(HyperDriveSubsystem hyperDrive) {
        return Commands.either(
            // If ready: run the engage command
            engage(hyperDrive),
            // If not ready: do nothing
            Commands.none(),
            // Condition: check isReady()
            () -> hyperDrive.isReady()
        );
    }
    
    // ========== EXAMPLE BUTTON BINDINGS ==========
    // This shows how these commands would be used in RobotContainer
    // (This is documentation, not actual code that runs)
    
    /*
     * In RobotContainer.java, you'd wire up buttons like this:
     * 
     * private void configureBindings() {
     *     // Hold A to charge hyperdrive
     *     driver.a().whileTrue(HyperDriveCommands.charge(hyperDrive));
     *     
     *     // Press X to charge until ready (auto-stops)
     *     driver.x().onTrue(HyperDriveCommands.chargeUntilReady(hyperDrive));
     *     
     *     // Hold right trigger to engage (go to hyperspace!)
     *     driver.rightTrigger().whileTrue(HyperDriveCommands.engageIfReady(hyperDrive));
     *     
     *     // Press B for emergency stop (any time!)
     *     driver.b().onTrue(HyperDriveCommands.emergencyStop(hyperDrive));
     * }
     */
}
