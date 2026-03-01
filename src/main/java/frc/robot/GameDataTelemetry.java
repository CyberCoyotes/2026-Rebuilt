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

  // ========================================================================
    // SHIFT TELEMETRY EXPANSION PLAN (PSEUDOCODE ONLY - NOT ACTIVE CODE)
    // ------------------------------------------------------------------------
    // Place this block here (class scope, near publishers/state) when you
    // implement shift-aware Elastic telemetry.
    //
    // 1) Add new publishers under /GameData (or /ShiftData):
    //    - MatchTimeSec (double)
    //    - CurrentShift (int)
    //    - CurrentShiftColor (string: "RED" / "BLUE" / "UNKNOWN")
    //    - IsRedShift (boolean)
    //    - IsBlueShift (boolean)
    //
    // 2) Add shift constants (fill with official game timing):
    //    - SHIFT_2_START_SEC
    //    - SHIFT_3_START_SEC
    //    - SHIFT_4_START_SEC
    //
    // 3) Add helper methods:
    //    - computeShiftNumber(matchTimeSec)
    //    - computeShiftColor(inactiveFirstAlliance, shiftNumber)
    //
    // 4) In update():
    //    - Read matchTime = DriverStation.getMatchTime()
    //    - Compute current shift from time
    //    - Determine active color by shift and winner
    //      (winner active in shifts 2 & 4; other alliance active in 1 & 3)
    //    - Publish all shift topics for Elastic widgets
    //
    // 5) For "assume blue won auton" testing mode:
    //    - Optionally force inactiveFirstAlliance = BLUE during bring-up
    //    - Keep clearly marked as temporary test behavior
    // ========================================================================


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

// [PSEUDOCODE INSERT LOCATION]
    // Add shift publishers here when implementing:
    // private final DoublePublisher matchTimePublisher;
    // private final IntegerPublisher currentShiftPublisher;
    // private final StringPublisher currentShiftColorPublisher;
    // private final BooleanPublisher isRedShiftPublisher;
    // private final BooleanPublisher isBlueShiftPublisher;

    private InactiveAlliance inactiveFirstAlliance = InactiveAlliance.NONE;
    private boolean dataReceived = false;

    public GameDataTelemetry() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        gameDataTable = inst.getTable("GameData");

        inactiveAlliancePublisher = gameDataTable.getStringTopic("InactiveFirstAlliance").publish();
        isRedPublisher = gameDataTable.getBooleanTopic("IsRed").publish();
        isBluePublisher = gameDataTable.getBooleanTopic("IsBlue").publish();
        dataReceivedPublisher = gameDataTable.getBooleanTopic("DataReceived").publish();

        // [PSEUDOCODE INSERT LOCATION - CONSTRUCTOR]
        // Initialize shift publishers here:
        // matchTimePublisher = gameDataTable.getDoubleTopic("MatchTimeSec").publish();
        // currentShiftPublisher = gameDataTable.getIntegerTopic("CurrentShift").publish();
        // currentShiftColorPublisher = gameDataTable.getStringTopic("CurrentShiftColor").publish();
        // isRedShiftPublisher = gameDataTable.getBooleanTopic("IsRedShift").publish();
        // isBlueShiftPublisher = gameDataTable.getBooleanTopic("IsBlueShift").publish();

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
        // [PSEUDOCODE INSERT LOCATION - START OF UPDATE]
        // double matchTimeSec = DriverStation.getMatchTime();
        // int shiftNumber = computeShiftNumber(matchTimeSec);

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

        // [PSEUDOCODE INSERT LOCATION - AFTER GAME DATA PARSE]
        // String shiftColor = computeShiftColor(inactiveFirstAlliance, shiftNumber);
        // publishShiftState(matchTimeSec, shiftNumber, shiftColor);

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

        // [PSEUDOCODE INSERT LOCATION - SHIFT PUBLISH]
        // matchTimePublisher.set(matchTimeSec);
        // currentShiftPublisher.set(shiftNumber);
        // currentShiftColorPublisher.set(shiftColor);
        // isRedShiftPublisher.set("RED".equals(shiftColor));
        // isBlueShiftPublisher.set("BLUE".equals(shiftColor));
    }

    // [PSEUDOCODE INSERT LOCATION - HELPER METHODS]
    // private int computeShiftNumber(double matchTimeSec) {
    //     if (matchTimeSec > SHIFT_2_START_SEC) return 1;
    //     if (matchTimeSec > SHIFT_3_START_SEC) return 2;
    //     if (matchTimeSec > SHIFT_4_START_SEC) return 3;
    //     return 4;
    // }
    //
    // private String computeShiftColor(InactiveAlliance winner, int shiftNumber) {
    //     if (winner == InactiveAlliance.NONE) return "UNKNOWN";
    //
    //     // Winner alliance is active in shifts 2 and 4.
    //     boolean winnerActive = (shiftNumber == 2 || shiftNumber == 4);
    //
    //     if (winner == InactiveAlliance.BLUE) {
    //         return winnerActive ? "BLUE" : "RED";
    //     }
    //     return winnerActive ? "RED" : "BLUE";
    // }

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
