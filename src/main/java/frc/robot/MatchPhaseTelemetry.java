package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Tracks the current match phase and publishes a countdown to the next phase change.
 *
 * Match timeline per the 2026 game manual (Table 6-2):
 *   Auto        20s   (FMS auto period, matchTime 20→0)
 *   Transition  10s   (TELEOP matchTime 140→130)
 *   Shift 1     25s   (TELEOP matchTime 130→105)
 *   Shift 2     25s   (TELEOP matchTime 105→80)
 *   Shift 3     25s   (TELEOP matchTime 80→55)
 *   Shift 4     25s   (TELEOP matchTime 55→30)
 *   Endgame     30s   (TELEOP matchTime 30→0)
 *
 * Both hubs are active during Auto, Transition, and Endgame.
 * During Shifts 1-4, hubs alternate based on auto scoring result.
 *
 * NetworkTables (/MatchPhase/):
 *   PhaseName      (string) – current phase label
 *   PhaseCountdown (double) – seconds until next phase begins
 */
public class MatchPhaseTelemetry {

    // TELEOP matchTime boundaries (seconds remaining in the 140s teleop period)
    private static final double TRANSITION_END = 130.0; // 140 - 10
    private static final double SHIFT1_END     = 105.0; // 130 - 25
    private static final double SHIFT2_END     =  80.0; // 105 - 25
    private static final double SHIFT3_END     =  55.0; //  80 - 25
    private static final double SHIFT4_END     =  30.0; //  55 - 25
    // Endgame: 30 → 0

    private final StringPublisher phaseNamePublisher;
    private final DoublePublisher phaseCountdownPublisher;

    public MatchPhaseTelemetry() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("MatchPhase");
        phaseNamePublisher      = table.getStringTopic("PhaseName").publish();
        phaseCountdownPublisher = table.getDoubleTopic("PhaseCountdown").publish();

        phaseNamePublisher.set("Pre-Match");
        phaseCountdownPublisher.set(0.0);
    }

    /**
     * Call this every robot periodic loop.
     */
    public void update() {
        String phase;
        double countdown;

        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            phase = "Auto";
            countdown = Math.max(0.0, matchTime);

        } else if (DriverStation.isTeleop() && DriverStation.isEnabled()) {
            if (matchTime < 0) {
                // FMS not sending time — fall back gracefully
                phase = "Teleop";
                countdown = 0.0;
            } else if (matchTime > TRANSITION_END) {
                phase = "Transition";
                countdown = matchTime - TRANSITION_END;
            } else if (matchTime > SHIFT1_END) {
                phase = "Shift 1";
                countdown = matchTime - SHIFT1_END;
            } else if (matchTime > SHIFT2_END) {
                phase = "Shift 2";
                countdown = matchTime - SHIFT2_END;
            } else if (matchTime > SHIFT3_END) {
                phase = "Shift 3";
                countdown = matchTime - SHIFT3_END;
            } else if (matchTime > SHIFT4_END) {
                phase = "Shift 4";
                countdown = matchTime - SHIFT4_END;
            } else {
                phase = "Endgame";
                countdown = Math.max(0.0, matchTime);
            }

        } else {
            phase = DriverStation.isEnabled() ? "Test" : "Pre-Match";
            countdown = 0.0;
        }

        phaseNamePublisher.set(phase);
        phaseCountdownPublisher.set(countdown);
    }
}
