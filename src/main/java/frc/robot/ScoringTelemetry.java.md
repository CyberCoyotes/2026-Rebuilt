package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/**
 * ScoringTelemetry - Publishes scoring status to NetworkTables for Elastic Dashboard.
 *
 * This class tracks whether the Hub/scoring target is active and ready to accept game pieces.
 * The dashboard displays this as a visual indicator for drivers.
 *
 * USAGE:
 * 1. Create instance in RobotContainer
 * 2. Call setHubActive(true/false) based on game state or sensor feedback
 * 3. Call update() in robotPeriodic() or from a subsystem
 *
 * NETWORKTABLES STRUCTURE:
 * /Scoring/
 *   - HubActive (boolean) - Is the hub ready to accept game pieces
 *   - Status (string) - Human-readable status message
 */
public class ScoringTelemetry {

    private final NetworkTable scoringTable;
    private final BooleanPublisher hubActivePublisher;
    private final StringPublisher statusPublisher;

    private boolean hubActive = false;
    private String statusMessage = "Initializing...";

    public ScoringTelemetry() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        scoringTable = inst.getTable("Scoring");

        hubActivePublisher = scoringTable.getBooleanTopic("HubActive").publish();
        statusPublisher = scoringTable.getStringTopic("Status").publish();

        // Initialize with default values
        update();
    }

    /**
     * Sets whether the Hub is active and ready to accept game pieces.
     *
     * @param active true if hub is accepting game pieces, false otherwise
     */
    public void setHubActive(boolean active) {
        this.hubActive = active;
    }

    /**
     * Sets a custom status message to display on the dashboard.
     *
     * @param message Status message (e.g., "Ready to Score", "Hub Full", "Aligning...")
     */
    public void setStatus(String message) {
        this.statusMessage = message;
    }

    /**
     * Convenience method to set both hub active state and status message.
     *
     * @param active true if hub is active
     * @param message Status message to display
     */
    public void setHubStatus(boolean active, String message) {
        this.hubActive = active;
        this.statusMessage = message;
    }

    /**
     * Returns whether the hub is currently active.
     *
     * @return true if hub is active
     */
    public boolean isHubActive() {
        return hubActive;
    }

    /**
     * Publishes current scoring state to NetworkTables.
     * Call this in robotPeriodic() or from a subsystem's periodic().
     */
    public void update() {
        hubActivePublisher.set(hubActive);
        statusPublisher.set(statusMessage);
    }
}
