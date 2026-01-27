package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * GameDataTelemetry - Accesses FMS game data and publishes scoring shift information to NetworkTables.
 *
 * In the 2026 game, the first goal to go inactive is determined by the alliance that scores more
 * Fuel in Auto. The field transmits this as a single character ('R' or 'B') approximately 3 seconds
 * after Auto ends.
 *
 * USAGE:
 * 1. Create instance in RobotContainer
 * 2. Call update() in robotPeriodic() to continuously poll for game data
 * 3. Use getInactiveFirstAlliance() to access the data programmatically
 *
 * NETWORKTABLES STRUCTURE:
 * /GameData/
 *   - InactiveFirstAlliance (string) - 'R', 'B', or 'NONE' (not yet received)
 *   - IsRed (boolean) - true if Red alliance's goal goes inactive first
 *   - IsBlue (boolean) - true if Blue alliance's goal goes inactive first
 *   - DataReceived (boolean) - true once valid game data has been received
 *
 * DASHBOARD USAGE:
 * - Use IsRed with a Boolean Box (red color when true) to show Red indicator
 * - Use IsBlue with a Boolean Box (blue color when true) to show Blue indicator
 * - Both will be false until data is received from FMS (~3 seconds after Auto)
 */
public class GameDataTelemetry {

    /** Represents which alliance's goal goes inactive first */
    public enum InactiveAlliance {
        NONE,  // Data not yet received
        RED,   // Red alliance goal inactive first (active in Shifts 2 and 4)
        BLUE   // Blue alliance goal inactive first (active in Shifts 2 and 4)
    }

    private final NetworkTable gameDataTable;
    private final StringPublisher inactiveAlliancePublisher;
    private final BooleanPublisher isRedPublisher;
    private final BooleanPublisher isBluePublisher;
    private final BooleanPublisher dataReceivedPublisher;

    private InactiveAlliance inactiveFirstAlliance = InactiveAlliance.NONE;
    private boolean dataReceived = false;

    public GameDataTelemetry() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        gameDataTable = inst.getTable("GameData");

        inactiveAlliancePublisher = gameDataTable.getStringTopic("InactiveFirstAlliance").publish();
        isRedPublisher = gameDataTable.getBooleanTopic("IsRed").publish();
        isBluePublisher = gameDataTable.getBooleanTopic("IsBlue").publish();
        dataReceivedPublisher = gameDataTable.getBooleanTopic("DataReceived").publish();

        // Initialize with default values
        update();
    }

    /**
     * Polls for game data from FMS and publishes to NetworkTables.
     * Call this in robotPeriodic() to continuously check for game data.
     *
     * Data becomes available approximately 3 seconds after Auto ends.
     * Until then, getGameSpecificMessage() returns an empty string.
     */
    public void update() {
        // Only poll if we haven't received data yet (data doesn't change during a match)
        if (!dataReceived) {
            String gameData = DriverStation.getGameSpecificMessage();

            if (gameData.length() > 0) {
                switch (gameData.charAt(0)) {
                    case 'R':
                        inactiveFirstAlliance = InactiveAlliance.RED;
                        dataReceived = true;
                        break;
                    case 'B':
                        inactiveFirstAlliance = InactiveAlliance.BLUE;
                        dataReceived = true;
                        break;
                    default:
                        // Corrupt data - keep waiting
                        break;
                }
            }
        }

        // Publish current state to NetworkTables
        publishState();
    }

    /**
     * Publishes the current game data state to NetworkTables.
     */
    private void publishState() {
        // Publish string representation
        switch (inactiveFirstAlliance) {
            case RED:
                inactiveAlliancePublisher.set("R");
                break;
            case BLUE:
                inactiveAlliancePublisher.set("B");
                break;
            default:
                inactiveAlliancePublisher.set("NONE");
                break;
        }

        // Publish boolean flags for dashboard indicators
        isRedPublisher.set(inactiveFirstAlliance == InactiveAlliance.RED);
        isBluePublisher.set(inactiveFirstAlliance == InactiveAlliance.BLUE);
        dataReceivedPublisher.set(dataReceived);
    }

    /**
     * Returns which alliance's goal goes inactive first.
     *
     * @return The alliance whose goal is inactive first, or NONE if data not yet received
     */
    public InactiveAlliance getInactiveFirstAlliance() {
        return inactiveFirstAlliance;
    }

    /**
     * Returns whether valid game data has been received from FMS.
     *
     * @return true if game data has been received
     */
    public boolean isDataReceived() {
        return dataReceived;
    }

    /**
     * Returns whether Red alliance's goal goes inactive first.
     *
     * @return true if Red goal inactive first, false otherwise (including if no data)
     */
    public boolean isRedInactiveFirst() {
        return inactiveFirstAlliance == InactiveAlliance.RED;
    }

    /**
     * Returns whether Blue alliance's goal goes inactive first.
     *
     * @return true if Blue goal inactive first, false otherwise (including if no data)
     */
    public boolean isBlueInactiveFirst() {
        return inactiveFirstAlliance == InactiveAlliance.BLUE;
    }

    /**
     * Resets the game data state. Call this at the start of a new match if needed.
     * Typically not required as data persists for the duration of a match.
     */
    public void reset() {
        inactiveFirstAlliance = InactiveAlliance.NONE;
        dataReceived = false;
        publishState();
    }
}
