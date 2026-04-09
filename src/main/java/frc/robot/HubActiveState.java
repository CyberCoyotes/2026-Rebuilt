package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubActiveState {
    private HubActiveState(){}

    static HubActiveState _inst = new HubActiveState();
    public static HubActiveState getInstance(){ return _inst; }

    private class TimeSegment {
        public double start;
        public double end;
        public TimeSegment(double start, double end) {
            this.start = start;
            this.end = end;
        }
        public boolean isTimeWithin(double time) {
            return time >= start && time <= end;
        }
    }
    // Teleop counts down from -160s to 0s, so these times are all based on that. These are the times that our hub is active during teleop based on if we won or lost auto, which we get from the DS message. The reason for the different times is to give teams a chance to swap out their auto and teleop routines if they won or lost auto, since that can impact what they want to do in teleop. The times are based on the manual, but also take into account that shift 4 and end game overlap, so we start a little earlier to give teams a chance to react to that.
    //Winner active: Shift 2 (90-120s), Shift 4 (25-55s), Transition Shift (130-140s)
    private final TimeSegment[] TeleopHubActiveTimesForAutoWinner = new TimeSegment[] {
        new TimeSegment(0,   60),  // Shift 4 (30-60s) + Endgame (0-30s)
        new TimeSegment(90, 120),  // Shift 2
    };
    // Loser active: Shift 3 (20-50s), Shift 1 (80-110s), Transition Shift (130-140s)
    private final TimeSegment[] TeleopHubActiveTimesForAutoLoser = new TimeSegment[] {
        new TimeSegment(60, 90),   // Shift 3
        new TimeSegment(120, 150),  // Shift 1
    };

    /* True when our hub is active */
    private boolean isHubActive = false;
    private double timeUntilTransition = 0;
    
    /* What to publish over networktables for telemetry */
    private final NetworkTable inst = NetworkTableInstance.getDefault().getTable("Hub");
    private final BooleanPublisher isHubActivePublisher = inst.getBooleanTopic("Active").publish();
    private final DoublePublisher timeUntilSwapPublisher = inst.getDoubleTopic("Time Until Swap").publish();

    /* Hoot auto-log and auto-replay support */
    private final HootAutoReplay autoReplay = new HootAutoReplay()
        .withBoolean("Hub/Active", () -> isHubActive, val -> isHubActive = val.value)
        .withDouble("Hub/Time Until Swap", () -> timeUntilTransition, val -> timeUntilTransition = val.value);

    private void updateStatesForTeleop() {
        if (!DriverStation.isTeleopEnabled()) return;

        String whoWonAuto = DriverStation.getGameSpecificMessage();
        boolean redIsWinner;
        switch (whoWonAuto) {
            case "R": redIsWinner = true; break;
            case "B": redIsWinner = false; break;
            default: return; // We don't have the message yet, we can't determine
        }

        /* Check to see if our current time is within a time that it doesn't matter what alliance we're on */
        double timeLeftInTeleop = DriverStation.getMatchTime();
        double timeUntilSwap = 150; // Start at 2:30 so it definitely gets cleared

        /* When we're in teleop we should definitely have the alliance color, so we can confidently use the DS alliance API */
        boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

        /* This boils down to an XOR but we explicitly call out both cases so it's clear */
        boolean useWinnerTimes = (redIsWinner && isRedAlliance) || (!redIsWinner && !isRedAlliance);

        /* If we're the winner, use the winner times */
        if (useWinnerTimes) {
            for (TimeSegment seg : TeleopHubActiveTimesForAutoWinner) {
                if (seg.isTimeWithin(timeLeftInTeleop)) {
                    timeUntilTransition = timeLeftInTeleop - seg.start;
                    isHubActive = true;
                    return;
                }
                double timeToStart = timeLeftInTeleop - seg.end;
                if (timeToStart > 0 && timeToStart < timeUntilSwap) {
                    /* Update our time until swap with this, since it's sooner */
                    timeUntilSwap = timeToStart;
                }
            }
        } else {
            for (TimeSegment seg : TeleopHubActiveTimesForAutoLoser) {
                if (seg.isTimeWithin(timeLeftInTeleop)) {
                    timeUntilTransition = timeLeftInTeleop - seg.start;
                    isHubActive = true;
                    return;
                }
                double timeToStart = timeLeftInTeleop - seg.end;
                if (timeToStart > 0 && timeToStart < timeUntilSwap) {
                    /* Update our time until swap with this, since it's sooner */
                    timeUntilSwap = timeToStart;
                }
            }
        }
        timeUntilTransition = timeUntilSwap;
        isHubActive = false;
    }

    private void fetchInputs() {
        /* Check what state we're in */
        if (DriverStation.isDisabled()) {
            /* If we're disabled, the hub is always inactive */
            isHubActive = false;
            timeUntilTransition = -1;
        } else if (DriverStation.isAutonomous()) {
            /* If we're autonomous, the hub is always active */
            isHubActive = true;
            timeUntilTransition = DriverStation.getMatchTime();
        } else if (DriverStation.isTeleop()) {
            /* If we're teleop, the hub will go through phases as outlined in the manual that we'll use to determine the active state */
            updateStatesForTeleop();
        } else {
            /* This is a state that we don't recognize, so make ti false */
            isHubActive = false;
        }
    }

    public void periodic() {
        /* If we're not replaying, then fetch data, otherwise let the autoreplay handle filling in data */
        if (!Utils.isReplay()) {
            fetchInputs();
        }
        autoReplay.update();

        isHubActivePublisher.accept(isHubActive);
        timeUntilSwapPublisher.accept(timeUntilTransition);
    }

    public boolean isOurHubActive() {
        return isHubActive;
    }

    public double timeUntilTransition() {
        return timeUntilTransition;
    }
}
