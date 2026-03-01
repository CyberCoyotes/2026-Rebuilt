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
    // SHIFT/FMS TELEMETRY REFERENCE IMPLEMENTATION (COMMENTED OUT ON PURPOSE)
    // ------------------------------------------------------------------------
    // WHY THIS BLOCK EXISTS:
    // - This is a "copy-when-ready" guide for implementing shift-aware telemetry.
    // - Everything stays commented so current robot behavior is unchanged.
    // - Future developers can uncomment in stages and keep a clean review diff.
    //
    // WHAT THIS FEATURE WOULD PUBLISH TO ELASTIC:
    //   /GameData/MatchTimeSec        (double)  -> match time from FMS/DS
    //   /GameData/CurrentShift        (int)     -> 1..4
    //   /GameData/CurrentShiftColor   (string)  -> "RED" / "BLUE" / "UNKNOWN"
    //   /GameData/IsRedShift          (boolean) -> true when RED is active shift
    //   /GameData/IsBlueShift         (boolean) -> true when BLUE is active shift
    //
    // GAME LOGIC ASSUMPTION USED:
    // - Winning auto alliance is active in shifts 2 and 4.
    // - Other alliance is active in shifts 1 and 3.
    // - If winner unknown, color is "UNKNOWN" and both booleans false.
    //
    // HOW TO ENABLE LATER (SAFE ORDER):
    // 1) Uncomment imports for DoublePublisher / IntegerPublisher.
    // 2) Uncomment field declarations for shift publishers.
    // 3) Uncomment constructor publisher initialization.
    // 4) Uncomment update() local vars + compute calls.
    // 5) Uncomment publishState(...) params and write calls.
    // 6) Uncomment helper methods + shift timing constants.
    // 7) Test in sim, then DS practice mode, then field.
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    // [IMPORTS TO ADD WHEN ENABLING]
    // import edu.wpi.first.networktables.DoublePublisher;
    // import edu.wpi.first.networktables.IntegerPublisher;
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    // [SHIFT TIMING CONSTANTS - FILL FROM GAME MANUAL]
    // NOTE: These are placeholders. Replace with official boundaries (seconds
    // remaining) that define where shifts transition in your match timeline.
    //
    // private static final double SHIFT_2_START_SEC = 90.0;
    // private static final double SHIFT_3_START_SEC = 60.0;
    // private static final double SHIFT_4_START_SEC = 30.0;
    // ------------------------------------------------------------------------

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

    // ------------------------------------------------------------------------
    // [SHIFT PUBLISHERS - UNCOMMENT TO ENABLE]
    // Keep these with the other publisher fields at class scope.
    // private final DoublePublisher matchTimePublisher;
    // private final IntegerPublisher currentShiftPublisher;
    // private final StringPublisher currentShiftColorPublisher;
    // private final BooleanPublisher isRedShiftPublisher;
    // private final BooleanPublisher isBlueShiftPublisher;
    // ------------------------------------------------------------------------

    private InactiveAlliance inactiveFirstAlliance = InactiveAlliance.NONE;
    private boolean dataReceived = false;

    public GameDataTelemetry() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        gameDataTable = inst.getTable("GameData");

        inactiveAlliancePublisher = gameDataTable.getStringTopic("InactiveFirstAlliance").publish();
        isRedPublisher = gameDataTable.getBooleanTopic("IsRed").publish();
        isBluePublisher = gameDataTable.getBooleanTopic("IsBlue").publish();
        dataReceivedPublisher = gameDataTable.getBooleanTopic("DataReceived").publish();

        // --------------------------------------------------------------------
        // [CONSTRUCTOR STEP - UNCOMMENT TO ENABLE SHIFT TOPICS]
        // Create publishers once, exactly like other NetworkTables publishers.
        // matchTimePublisher = gameDataTable.getDoubleTopic("MatchTimeSec").publish();
        // currentShiftPublisher = gameDataTable.getIntegerTopic("CurrentShift").publish();
        // currentShiftColorPublisher = gameDataTable.getStringTopic("CurrentShiftColor").publish();
        // isRedShiftPublisher = gameDataTable.getBooleanTopic("IsRedShift").publish();
        // isBlueShiftPublisher = gameDataTable.getBooleanTopic("IsBlueShift").publish();
        // --------------------------------------------------------------------

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
        // --------------------------------------------------------------------
        // [UPDATE STEP 1 - READ TIME + COMPUTE SHIFT NUMBER]
        // DriverStation.getMatchTime() is the current match time remaining.
        // double matchTimeSec = DriverStation.getMatchTime();
        // int shiftNumber = computeShiftNumber(matchTimeSec);
        // --------------------------------------------------------------------

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

        // --------------------------------------------------------------------
        // [UPDATE STEP 2 - MAP WINNER + SHIFT TO ACTIVE COLOR]
        // If data not received yet, this should produce "UNKNOWN".
        // String shiftColor = computeShiftColor(inactiveFirstAlliance, shiftNumber);
        //
        // OPTIONAL TEST OVERRIDE (bring-up only):
        // inactiveFirstAlliance = InactiveAlliance.BLUE;
        // dataReceived = true;
        // shiftColor = computeShiftColor(inactiveFirstAlliance, shiftNumber);
        // --------------------------------------------------------------------

        // Publish current state to NetworkTables
        // NOTE: when enabling shift telemetry, pass local values into publishState.
        // publishState(matchTimeSec, shiftNumber, shiftColor);
        // return;

        // Publish current state to NetworkTables
        publishState();
    }

    /**
     * Publishes the current game data state to NetworkTables.
     */
    // ------------------------------------------------------------------------
    // Current active method. Keep this signature until you are ready to enable
    // shift telemetry. Then migrate to the overloaded version below.
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

        // --------------------------------------------------------------------
        // [SHIFT PUBLISH STEP - ENABLE WHEN READY]
        // matchTimePublisher.set(matchTimeSec);
        // currentShiftPublisher.set(shiftNumber);
        // currentShiftColorPublisher.set(shiftColor);
        // isRedShiftPublisher.set("RED".equals(shiftColor));
        // isBlueShiftPublisher.set("BLUE".equals(shiftColor));
        // --------------------------------------------------------------------

    }

    // ------------------------------------------------------------------------
    // [FULL COMMENTED REFERENCE - ENABLE IN STAGES]
    //
    // 1) Overload publishState to include shift fields:
    // private void publishState(double matchTimeSec, int shiftNumber, String shiftColor) {
    //     // Existing values
    //     switch (inactiveFirstAlliance) {
    //         case RED:  inactiveAlliancePublisher.set("R"); break;
    //         case BLUE: inactiveAlliancePublisher.set("B"); break;
    //         default:   inactiveAlliancePublisher.set("NONE"); break;
    //     }
    //     isRedPublisher.set(inactiveFirstAlliance == InactiveAlliance.RED);
    //     isBluePublisher.set(inactiveFirstAlliance == InactiveAlliance.BLUE);
    //     dataReceivedPublisher.set(dataReceived);
    //
    //     // New shift values
    //     matchTimePublisher.set(matchTimeSec);
    //     currentShiftPublisher.set(shiftNumber);
    //     currentShiftColorPublisher.set(shiftColor);
    //     isRedShiftPublisher.set("RED".equals(shiftColor));
    //     isBlueShiftPublisher.set("BLUE".equals(shiftColor));
    // }
    //
    // 2) Compute shift from current match time:
    // private int computeShiftNumber(double matchTimeSec) {
    //     if (matchTimeSec > SHIFT_2_START_SEC) return 1;
    //     if (matchTimeSec > SHIFT_3_START_SEC) return 2;
    //     if (matchTimeSec > SHIFT_4_START_SEC) return 3;
    //     return 4;
    // }
    //
    // 3) Compute active alliance color from winner + shift index:
    // private String computeShiftColor(InactiveAlliance winner, int shiftNumber) {
    //     if (winner == InactiveAlliance.NONE) return "UNKNOWN";
    //
    //     boolean winnerActive = (shiftNumber == 2 || shiftNumber == 4);
    //
    //     if (winner == InactiveAlliance.BLUE) {
    //         return winnerActive ? "BLUE" : "RED";
    //     }
    //     return winnerActive ? "RED" : "BLUE";
    // }
    // ------------------------------------------------------------------------

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
