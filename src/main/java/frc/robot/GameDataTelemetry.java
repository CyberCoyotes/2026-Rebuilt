package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Reads FMS game data and publishes it to NetworkTables for the Elastic dashboard.
 *
 * In the 2026 game, the alliance that scores more Fuel in Auto determines which
 * goal goes inactive first. The field sends 'R' or 'B' ~3 seconds after Auto ends.
 *
 * NetworkTables (/GameData/):
 *   InactiveFirstAlliance  (string)  - "R", "B", or "NONE"
 *   IsRed                  (boolean) - true if Red goal goes inactive first
 *   IsBlue                 (boolean) - true if Blue goal goes inactive first
 *   DataReceived           (boolean) - true once FMS has sent valid data
 *   ActiveHub              (string)  - "RED", "BLUE", or "UNKNOWN"
 *   CurrentShift           (string)  - "Shift 1" through "Shift 4" or "UNKNOWN"
 *   TimeUntilShiftEnd      (double)  - seconds remaining until the current shift ends
 *   IsRedHubActive         (boolean) - true when Red hub is currently active
 *   IsBlueHubActive        (boolean) - true when Blue hub is currently active
 *
 * Dashboard widgets:
 *   - Boolean Box on IsRedHubActive  (color red)  -> lights up when Red hub is active
 *   - Boolean Box on IsBlueHubActive (color blue) -> lights up when Blue hub is active
 *   - Text Display on ActiveHub -> shows "RED" or "BLUE"
 *
<<<<<<< Updated upstream
=======
 * * Match structure (teleop counts DOWN from ~160s to 0):
 *   Transition  160s -> 150s  (10s)
 *   Shift 1     150s -> 120s  (30s)
 *   Shift 2     120s ->  90s  (30s)
 *   Shift 3      90s ->  60s  (30s)
 *   Shift 4      60s ->  30s  (30s)
 *   Endgame      30s ->   0s  (30s)
 * 
>>>>>>> Stashed changes
 * Shift logic (winner = InactiveFirstAlliance):
 *   Shifts 1 & 3 -> opposite alliance hub active
 *   Shifts 2 & 4 -> winner's hub active
 */
public class GameDataTelemetry {

<<<<<<< Updated upstream
    // NOTE: Fill these in from the game manual before competition.
    // These are seconds REMAINING in the match (DriverStation.getMatchTime() counts down).
    private static final double SHIFT_2_START_SEC = 90.0;
    private static final double SHIFT_3_START_SEC = 60.0;
    private static final double SHIFT_4_START_SEC = 30.0;

=======
   // Seconds REMAINING in teleop when each period begins (getMatchTime() counts down).
    private static final double TRANSITION_END_SEC  = 150.0; // transition ends, shift 1 begins
    private static final double SHIFT_2_START_SEC   = 120.0;
    private static final double SHIFT_3_START_SEC   =  90.0;
    private static final double SHIFT_4_START_SEC   =  60.0;
    private static final double ENDGAME_START_SEC   =  30.0;
   
>>>>>>> Stashed changes
    public enum InactiveAlliance {
        NONE,
        RED,
        BLUE
    }

    private final NetworkTable gameDataTable;
    private final StringPublisher inactiveAlliancePublisher;
    private final BooleanPublisher isRedPublisher;
    private final BooleanPublisher isBluePublisher;
    private final BooleanPublisher dataReceivedPublisher;
    private final StringPublisher activeHubPublisher;
    private final BooleanPublisher isRedHubActivePublisher;
    private final BooleanPublisher isBlueHubActivePublisher;
    private final StringPublisher currentShiftPublisher;
    private final DoublePublisher timeUntilShiftEndPublisher;

    private InactiveAlliance inactiveFirstAlliance = InactiveAlliance.NONE;
    private boolean dataReceived = false;
    private String activeHub = "UNKNOWN";

    public GameDataTelemetry() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        gameDataTable = inst.getTable("GameData");

        inactiveAlliancePublisher = gameDataTable.getStringTopic("InactiveFirstAlliance").publish();
        isRedPublisher = gameDataTable.getBooleanTopic("IsRed").publish();
        isBluePublisher = gameDataTable.getBooleanTopic("IsBlue").publish();
        dataReceivedPublisher = gameDataTable.getBooleanTopic("DataReceived").publish();
        activeHubPublisher = gameDataTable.getStringTopic("ActiveHub").publish();
        isRedHubActivePublisher = gameDataTable.getBooleanTopic("IsRedHubActive").publish();
        isBlueHubActivePublisher = gameDataTable.getBooleanTopic("IsBlueHubActive").publish();
        currentShiftPublisher = gameDataTable.getStringTopic("CurrentShift").publish();
        timeUntilShiftEndPublisher = gameDataTable.getDoubleTopic("TimeUntilShiftEnd").publish();

        update();
    }

    /**
     * Polls FMS for game data and publishes to NetworkTables.
     * Call this in robotPeriodic(). Data arrives ~3 seconds after Auto ends.
     */
    public void update() {

        
        // vvv CANCEL FOR COMPETITION - remove these two lines before competing vvv
        inactiveFirstAlliance = InactiveAlliance.BLUE;
        dataReceived = true;
        // ^^^ CANCEL FOR COMPETITION ^^^

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
                        // Unrecognized character - keep waiting
                        break;
                }
            }
        }

        double matchTimeSec = DriverStation.getMatchTime();
        int shift = computeShift(matchTimeSec);
        activeHub = computeActiveHub(shift);
        publishState(activeHub, shift, matchTimeSec);
    }

    private void publishState(String activeHub, int shift, double matchTimeSec) {
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

        isRedPublisher.set(inactiveFirstAlliance == InactiveAlliance.RED);
        isBluePublisher.set(inactiveFirstAlliance == InactiveAlliance.BLUE);
        dataReceivedPublisher.set(dataReceived);

        activeHubPublisher.set(activeHub);
        isRedHubActivePublisher.set("RED".equals(activeHub));
        isBlueHubActivePublisher.set("BLUE".equals(activeHub));
<<<<<<< Updated upstream
        currentShiftPublisher.set(shift > 0 ? "Shift " + shift : "UNKNOWN");
=======
       
        String shiftLabel;
        if (shift == -1)     shiftLabel = "UNKNOWN";
        else if (shift == 0) shiftLabel = "Transition";
        else if (shift == 5) shiftLabel = "Endgame";
        else                 shiftLabel = "Shift " + shift;
        currentShiftPublisher.set(shiftLabel);

>>>>>>> Stashed changes
        timeUntilShiftEndPublisher.set(computeTimeUntilShiftEnd(shift, matchTimeSec));
    }

    /**
<<<<<<< Updated upstream
     * Returns the current shift number (1-4) based on seconds remaining.
     * Returns 0 if match time is unavailable.
     */
=======
    * Returns the current period:
     *  -1 = match time unavailable
     *   0 = Transition (150-160s remaining)
     *   1 = Shift 1    (120-150s remaining)
     *   2 = Shift 2    ( 90-120s remaining)
     *   3 = Shift 3    ( 60- 90s remaining)
     *   4 = Shift 4    ( 30- 60s remaining)
     *   5 = Endgame    (  0- 30s remaining)
     *  */
>>>>>>> Stashed changes
    private int computeShift(double matchTimeSec) {
        if (matchTimeSec < 0) return 0;
        if (matchTimeSec > SHIFT_2_START_SEC) return 1;
        if (matchTimeSec > SHIFT_3_START_SEC) return 2;
        if (matchTimeSec > SHIFT_4_START_SEC) return 3;
        return 4;
    }

    /**
<<<<<<< Updated upstream
     * Returns seconds remaining until the current shift ends, or -1 if match time is unavailable.
=======
     * Returns seconds remaining until the current  period ends, or -1 if unavailable.
>>>>>>> Stashed changes
     */
    private double computeTimeUntilShiftEnd(int shift, double matchTimeSec) {
        if (matchTimeSec < 0) return -1;

        switch (shift) {
            case 1: return matchTimeSec - SHIFT_2_START_SEC;
            case 2: return matchTimeSec - SHIFT_3_START_SEC;
            case 3: return matchTimeSec - SHIFT_4_START_SEC;
            case 4: return matchTimeSec; // Shift 4 ends when match time hits 0
            default: return -1;
        }
    }
    /**
     * Returns which hub is currently active based on who won auto and the current shift.
     * The winner's hub is active in shifts 2 and 4; the other alliance in shifts 1 and 3.
     */
    private String computeActiveHub(int shift) {
        if (inactiveFirstAlliance == InactiveAlliance.NONE || shift == 0) return "UNKNOWN";

        boolean winnerActive = (shift == 2 || shift == 4);

        if (inactiveFirstAlliance == InactiveAlliance.BLUE) {
            return winnerActive ? "BLUE" : "RED";
        }
        return winnerActive ? "RED" : "BLUE";
    }

    /** Returns which alliance's goal goes inactive first, or NONE if not yet received. */
    public InactiveAlliance getInactiveFirstAlliance() {
        return inactiveFirstAlliance;
    }

    /** Returns true once valid game data has been received from FMS. */
    public boolean isDataReceived() {
        return dataReceived;
    }

    /** Returns true if Red alliance's goal goes inactive first. */
    public boolean isRedInactiveFirst() {
        return inactiveFirstAlliance == InactiveAlliance.RED;
    }

    /** Returns true if Blue alliance's goal goes inactive first. */
    public boolean isBlueInactiveFirst() {
        return inactiveFirstAlliance == InactiveAlliance.BLUE;
    }

    /** Returns true when the Red hub is currently the active scoring target. */
    public boolean isRedHubActive() {
        return "RED".equals(activeHub);
    }

    /** Returns true when the Blue hub is currently the active scoring target. */
    public boolean isBlueHubActive() {
        return "BLUE".equals(activeHub);
    }
}
