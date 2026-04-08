package frc.robot.subsystems.shooter;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

/**
 * STATE MACHINE:
 * - IDLE: All motors off, hood at home position
 * - STANDBY: Flywheel held at STANDBY_RPM (1800), hood at home — operator-toggled pre-rev mode
 * - SPINNING_UP: Flywheel ramping to target after trigger pull — transitions to READY when at speed
 * - READY: Flywheel and hood at preset targets, ready to shoot
 * - PASS: Passing shot at PASS_RPM, hood at PASS_HOOD
 * - EJECT: Flywheel reverse at EJECT_RPM for clearing jams — velocity-gated
 * - POPPER: Low-speed mode for assisted fuel loading
 *
 * SHOT FLOW:
 * - Driver RT alone → VisionShootCommand (default, auto-aims from distance)
 * - Operator holds A/B/X/Y + Driver RT → fires the corresponding named preset
 * - Operator holding a button shows the preset label on Elastic for sanity check
 *   (no trigger required to see the display)
 *
 */

public class ShooterSubsystem extends SubsystemBase {

    // =====================================================================
    // Shot Presets
    // =====================================================================

    /**
     * Named shot presets that can be selected via POV left/right on the driver controller.
     * The selected preset is set silently and fired on the right trigger.
     * Published to NetworkTables as Shooter/SelectedPreset for Elastic display.
     */
    public enum ShotPreset {
        CLOSE  ("Close",    Constants.Flywheel.CLOSE_RPM,  Constants.Hood.CLOSE_HOOD),
        TOWER  ("Tower",    Constants.Flywheel.TOWER_RPM,  Constants.Hood.TOWER_HOOD),
        TRENCH ("Trench",   Constants.Flywheel.TRENCH_RPM, Constants.Hood.TRENCH_HOOD),
        PASS   ("Pass",     Constants.Flywheel.PASS_RPM,   Constants.Hood.PASS_HOOD),
        FAR    ("Corner",   Constants.Flywheel.FAR_RPM,    Constants.Hood.FAR_HOOD),
        POPPER ("Popper",   Constants.Flywheel.POPPER_RPM, Constants.Hood.POPPER_HOOD);

        public final String label;
        public final double rpm;
        public final double hood;

        ShotPreset(String label, double rpm, double hood) {
            this.label = label;
            this.rpm   = rpm;
            this.hood  = hood;
        }
    }

    // =====================================================================
    // Hardware Interface
    // =====================================================================
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // =====================================================================
    // Dashboard Publishers
    // =====================================================================
    private final NetworkTable shooterTable;
    private final StringPublisher  statePublisher;
    private final BooleanPublisher readyPublisher;
    private final DoublePublisher  flywheelRpmPublisher;
    private final DoublePublisher  targetRpmPublisher;
    private final DoublePublisher  flywheelErrorPublisher;
    private final DoublePublisher  hoodRotationsPublisher;
    private final DoublePublisher  targetHoodRotationsPublisher;
    private final DoublePublisher  hoodErrorPublisher;
    private final BooleanPublisher hoodAtPosePublisher;
    private final BooleanPublisher flywheelAtRpmPublisher;
    private final StringPublisher  selectedPresetPublisher;
    private final BooleanPublisher standbyEnabledPublisher;

    // =====================================================================
    // State
    // =====================================================================
    private boolean      standbyEnabled   = false; // operator-toggled — does NOT default on at startup
    private ShooterState currentState     = ShooterState.IDLE;
    private ShotPreset   displayPreset    = null; // null = Vision mode (no operator button held)
    private String currentStateString     = ShooterState.IDLE.toString();
    // private boolean standbyEnabled        = false; // operator toggled, defaults OFF at boot
    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot      = 0.0;

    // Slow publish divider
    private int periodicCounter = 0;

    // =====================================================================
    // Constructor
    // =====================================================================
    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        shooterTable = inst.getTable("Shooter");

        statePublisher               = shooterTable.getStringTopic("State").publish();
        readyPublisher               = shooterTable.getBooleanTopic("IsReady").publish();
        flywheelRpmPublisher         = shooterTable.getDoubleTopic("FlywheelRPM").publish();
        targetRpmPublisher           = shooterTable.getDoubleTopic("TargetFlywheelRPM").publish();
        flywheelErrorPublisher       = shooterTable.getDoubleTopic("FlywheelError").publish();
        hoodRotationsPublisher       = shooterTable.getDoubleTopic("HoodRotations").publish();
        targetHoodRotationsPublisher = shooterTable.getDoubleTopic("TargetHoodRotations").publish();
        hoodErrorPublisher           = shooterTable.getDoubleTopic("HoodError").publish();
        hoodAtPosePublisher          = shooterTable.getBooleanTopic("HoodAtPose").publish();
        flywheelAtRpmPublisher       = shooterTable.getBooleanTopic("FlywheelAtRPM").publish();
        selectedPresetPublisher       = shooterTable.getStringTopic("SelectedPreset").publish();
        standbyEnabledPublisher       = shooterTable.getBooleanTopic("StandbyEnabled").publish();
    }

    // =====================================================================
    // State Machine
    // =====================================================================
    /* Defines what mode the shooter is in. Each value represents a distinct operational mode with different hardware behavior */
    public enum ShooterState {
        IDLE,        // Motors off, hood at home — default state, not used during active match play
        STANDBY,     // Flywheel held at STANDBY_RPM, hood at home — operator-toggled pre-rev
        SPINNING_UP, // Flywheel ramping to target after trigger pull — transitions to READY when at speed
        READY,   // Flywheel and hood at preset targets, ready to shoot
        PASS,    // Passing shot at PASS_RPM, hood at PASS_HOOD
        EJECT,   // Jam clearing: flywheel at EJECT_RPM (reverse), hood at MIN_POSE — velocity-gated
        POPPER   // Popper mode: flywheel and hood at minimum pose for assisting with fuel loading
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

    // SPINNING_UP → READY promotion
    // Only periodic() earns the READY state — never set directly
        if (currentState == ShooterState.SPINNING_UP
                && isFlywheelAtVelocity()
                && isHoodAtPose()) {
            setState(ShooterState.READY);
        }

        updateStateMachine();  // after promotion so READY takes effect this cycle

        if (++periodicCounter % 5 == 0) {
            io.updateSlowInputs(inputs);
            publishToElastic();
        }
}

    private void publishToElastic() {
        statePublisher.set(currentStateString);
        readyPublisher.set(isReady());
        flywheelRpmPublisher.set(inputs.flywheelLeaderMotorRPM);
        targetRpmPublisher.set(targetFlywheelMotorRPM);
        flywheelErrorPublisher.set(targetFlywheelMotorRPM - inputs.flywheelLeaderMotorRPM);
        hoodRotationsPublisher.set(inputs.hoodPositionRotations);
        targetHoodRotationsPublisher.set(targetHoodPoseRot);
        hoodErrorPublisher.set(targetHoodPoseRot - inputs.hoodPositionRotations);
        hoodAtPosePublisher.set(isHoodAtPose());
        flywheelAtRpmPublisher.set(isFlywheelAtVelocity());
        selectedPresetPublisher.set(displayPreset != null ? displayPreset.label : "Vision");
        standbyEnabledPublisher.set(standbyEnabled);
    }

    // State Machine Logic
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                break;
            case STANDBY:
                commandFlywheelVelocity(Constants.Flywheel.STANDBY_RPM);
                break;
            case SPINNING_UP:
                commandFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;
            case READY:
                commandFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;
            case PASS:
                commandFlywheelVelocity(Constants.Flywheel.PASS_RPM);
                io.setHoodPose(Constants.Hood.PASS_HOOD);
                break;
            case EJECT:
                commandFlywheelVelocity(Constants.Flywheel.EJECT_RPM);
                break;
            case POPPER:
                commandFlywheelVelocity(Constants.Flywheel.POPPER_RPM);
                io.setHoodPose(Constants.Hood.POPPER_HOOD);
                break;
        }
    }

    /* Handles transitions — called once when you want to change modes. It:
     Guards against no-op transitions (if currentState == newState return)
     Runs entry actions — one-time side effects that happen at the moment of entering a state (e.g. stopFlywheels() on entering IDLE, commandFlywheelVelocity() on entering PASS)
     */
    private void setState(ShooterState newState) {
        if (currentState == newState)
            return;

        currentState = newState;
        currentStateString = newState.toString();

        switch (newState) {
            case IDLE:
                io.stopFlywheels();
                io.setHoodPose(Constants.Hood.MIN_POSE);
                break;

            case STANDBY:
                commandFlywheelVelocity(Constants.Flywheel.STANDBY_RPM);
                io.setHoodPose(Constants.Hood.MIN_POSE);
                break;

            // The flywheel and hood targets set, and BOTH flywheel and hood are in progress to getting to those values and READY
            case SPINNING_UP:
                break;

            // case STANDBY:
            //     targetFlywheelMotorRPM = Constants.Flywheel.STANDBY_RPM;
            //     targetHoodPoseRot = Constants.Hood.MIN_POSE;
            //     commandFlywheelVelocity(Constants.Flywheel.STANDBY_RPM);
            //     io.setHoodPose(Constants.Hood.MIN_POSE);
            //     break;

            case READY:
                commandFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;

            case PASS:
                commandFlywheelVelocity(Constants.Flywheel.PASS_RPM);
                io.setHoodPose(Constants.Hood.PASS_HOOD);
                break;

            case EJECT:
                commandFlywheelVelocity(Constants.Flywheel.EJECT_RPM);
                io.setHoodPose(Constants.Hood.MIN_POSE);
                break;
            case POPPER:
                commandFlywheelVelocity(Constants.Flywheel.POPPER_RPM);
                io.setHoodPose(Constants.Hood.POPPER_HOOD);
                break;
        }
    }

    // =====================================================================
    // Public Command Methods
    // =====================================================================

    /**
     * Returns shooter to its resting state after a shot.
     * If standby mode is enabled, holds flywheel at STANDBY_RPM instead of full stop.
     * This is the method called by all finallyDo() shot endings.
     */
    public void setIdle() {
        if (standbyEnabled) {
            setState(ShooterState.STANDBY);
        } else {
            setState(ShooterState.IDLE);
        }
    }

    /**
     * Toggles standby pre-rev mode on/off.
     * - Toggle ON:  immediately enters STANDBY (flywheel spins up to STANDBY_RPM).
     * - Toggle OFF: drops to IDLE if currently in STANDBY; flag clears so next
     *               shot ending returns to IDLE instead of STANDBY.
     * Bound to operator Start button — operator sets it and forgets it.
     */
    public void toggleStandby() {
        standbyEnabled = !standbyEnabled;
        if (standbyEnabled) {
            if (currentState == ShooterState.IDLE) {
                setState(ShooterState.STANDBY);
            }
        } else {
            if (currentState == ShooterState.STANDBY) {
                setState(ShooterState.IDLE);
            }
        }
    }

    // /** Returns true if standby pre-rev mode is currently enabled. */
    // public boolean isStandbyEnabled() {
    //     return standbyEnabled;
    // }

    /** Holds the flywheel at standby RPM and homes the hood. */
    public void setStandby() {
        setState(ShooterState.STANDBY);
    }

    /** Teleop shot-end behavior: standby when enabled, otherwise full idle. */
    public void setPostShotState() {
        if (standbyEnabled) {
            setStandby();
        } else {
            setIdle();
        }
    }

    /** Toggle standby mode ON/OFF from an operator button. */
    public void toggleStandbyMode() {
        standbyEnabled = !standbyEnabled;
        if (standbyEnabled && currentState == ShooterState.IDLE) {
            setStandby();
        } else if (!standbyEnabled && currentState == ShooterState.STANDBY) {
            setIdle();
        }
    }

    public boolean isStandbyEnabled() {
        return standbyEnabled;
    }

    /**
     * Transitions state machine to SPINNING_UP.
     * Void method — safe to call inside lambdas, runOnce, and Commands.run() loops.
     * Use spinUpCommand() when you need a scheduled Command in a sequence.
     */
    public void beginSpinUp() {
        setState(ShooterState.SPINNING_UP);
    }

    /** Sets shooter to passing mode. */
    public void pass() {
        setState(ShooterState.PASS);
    }

    /**
     * Starts ejecting to clear jams.
     * Blocked if flywheel is above EJECT_MAX_ENTRY_RPM to prevent violent reversal.
     */
    public void eject() {
        if (Math.abs(getCurrentVelocityRPM()) > Constants.Flywheel.EJECT_MAX_ENTRY_RPM) {
            return; // Flywheel spinning too fast to safely reverse — refuse eject
        }
        setState(ShooterState.EJECT);
    }

    // Display Preset

    /**
     * Called when operator holds a preset button — shows the preset label on Elastic.
     * Does NOT change shooter state or target RPM/hood. Driver trigger still required to fire.
     */
    public void setDisplayPreset(ShotPreset preset) {
        displayPreset = preset;
    }

    /** Called when operator releases the preset button — reverts display to "Vision". */
    public void clearDisplayPreset() {
        displayPreset = null;
    }

    /**
     * Enters POPPER state directly — vision/distance updates cannot override this.
     */
    public void setAirPopper() {
        setState(ShooterState.POPPER);
    }

    // Target Setters

    /** Sets target flywheel velocity (forward only — clamped to MAX_FLYWHEEL_RPM). Does NOT change state. */
    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), Constants.Flywheel.MAX_RPM);
    }

    /**
     * Sets target hood pose in rotations. Clamped to valid range. Does NOT change state.
     */
    public void setTargetHoodPose(double rotations) {
        targetHoodPoseRot = Math.max(Constants.Hood.MIN_POSE, Math.min(Constants.Hood.MAX_POSE, rotations));
    }

    /** Adjusts target flywheel velocity by a delta. */
    public void adjustTargetVelocity(double deltaRPM) {
        setTargetVelocity(targetFlywheelMotorRPM + deltaRPM);
    }

    /**
     * Adjusts target hood pose by a delta in rotations.
     * If already in READY state, commands hood to move immediately.
     */
    public void adjustTargetHoodPose(double deltaRotations) {
        setTargetHoodPose(targetHoodPoseRot + deltaRotations);
        if (currentState == ShooterState.READY) {
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    /**
     * Updates hood position directly while in READY state.
     * Used by FarShotCommand to continuously adjust hood based on live distance.
     * Only takes effect if currently in READY state — otherwise silently updates target.
     *
     * @param rotations Target hood position in rotations
     */
    public void updateHoodForDistance(double rotations) {
        setTargetHoodPose(rotations);
        if (currentState == ShooterState.READY) {
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    // =====================================================================
    // Status Queries
    // =====================================================================

    /** Returns true if in READY state with flywheel and hood at targets. */
    public boolean isReady() {
        return currentState == ShooterState.READY && isFlywheelAtVelocity() && isHoodAtPose();
    }

    // isBusy() becomes possible
    public boolean isSpinningUp() {
        return currentState == ShooterState.SPINNING_UP;
    }

    /** Returns true if in PASS state with flywheel and hood at targets. */
    public boolean isPassReady() {
        return currentState == ShooterState.PASS && isFlywheelAtVelocity() && isHoodAtPose();
    }

    /** Returns true if flywheel is at target velocity within tolerance. */
    public boolean isFlywheelAtVelocity() {
        if (Math.abs(targetFlywheelMotorRPM) < 1.0) {
            return Math.abs(inputs.flywheelLeaderMotorRPM) < 50.0;
        }
        double tolerance = Math.abs(targetFlywheelMotorRPM) * Constants.Flywheel.TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    /** Returns true if hood is at target pose within tolerance. */
    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodPositionRotations - targetHoodPoseRot) < Constants.Hood.TOLERANCE_POSE;
    }

    /** Returns true if total flywheel current is too high. */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0;
    }

    // =====================================================================
    // Commands
    // =====================================================================

    /** Returns shooter to idle. */
    public Command idleCommand() {
        return Commands.runOnce(this::setIdle, this).withName("ShooterIdle");
    }

    public ShooterState getState() {
        return currentState;
    }

    public double getCurrentVelocityRPM() {
        return inputs.flywheelLeaderMotorRPM;
    }

    public double getCurrentHoodPose() {
        return inputs.hoodPositionRotations;
    }

    public double getTargetVelocityRPM() {
        return targetFlywheelMotorRPM;
    }

    public double getTargetHoodPose() {
        return targetHoodPoseRot;
    }

    /**
     * Command that holds SPINNING_UP state while scheduled and returns to IDLE on end.
     * Use as a standalone step in a Commands.sequence() — never call this inside a lambda.
     * For lambdas/runOnce/Commands.run(), use beginSpinUp() (void) instead.
     */
    public Command spinUpCommand() {
        return Commands.startEnd(
            () -> setState(ShooterState.SPINNING_UP),
            () -> setState(ShooterState.IDLE),
            this).withName("Shooter: SpinUpCommand");
    }

    // =====================================================================
    // Vision Lookup Tables
    // =====================================================================

    /**
     * Distance-to-RPM and distance-to-hood lookup tables.
     *
     * Keys   = distance from hub AprilTag in meters (floor distance, not slant).
     * Values = target flywheel RPM / hood rotations at that distance.
     *
     * InterpolatingDoubleTreeMap linearly interpolates between measured points and
     * clamps to the nearest endpoint outside the measured range.
     *
     * HOW TO FILL IN: See TUNING.md §4 for the full measurement procedure.
     *   1. Place robot at each distance with a tape measure.
     *   2. Enable, hold RT, watch Vision/Distance_m on Elastic to confirm distance reads correctly.
     *   3. Manually command a shot (use POV preset cycling as a baseline).
     *   4. Adjust RPM/hood until shots land center target.
     *   5. Record values here, rebuild, repeat at next distance.
     *
     * Distances below 1.0 m and above 6.0 m are clamped to the nearest endpoint.
     * Add or remove rows as the shot envelope changes.
     */
    private static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HOOD_ROT_MAP     = new InterpolatingDoubleTreeMap();

    static {
        /* Both maps MUST have identical distance keys — they are co-indexed.
        * Adding a distance to one map without adding it to the other produces
        * inconsistent RPM/hood pairings at that distance. Always update both.
        */
        FLYWHEEL_RPM_MAP.put(2.6, 2000.0);
        FLYWHEEL_RPM_MAP.put(3.50, 2250.0);
        FLYWHEEL_RPM_MAP.put(4.50,  3000.0);
        
        HOOD_ROT_MAP.put(2.6,  5.50);
        HOOD_ROT_MAP.put(3.50, 4.65);
        HOOD_ROT_MAP.put(4.50,  3.50);
    }

    /**
     * Updates shooter targets from the interpolation maps for the given distance.
     *
     * If the shooter is already in READY state the new targets are pushed to
     * hardware immediately, enabling continuous live tracking while the trigger
     * is held.  If not yet in READY, the targets are stored and applied when
     * prepareToShoot() is called.
     *
     * @param distanceMeters Measured distance to hub tag in meters (floor distance)
     */
    public void updateFromDistance(double distanceMeters) {
        // Never let vision override dedicated shot modes
        if (currentState == ShooterState.POPPER
                || currentState == ShooterState.EJECT
                || currentState == ShooterState.PASS) {
            return;
        }
        double dist = Math.max(3.0, Math.min(8.0, distanceMeters)); // max bumped to 8, minimum set to 3, as of 4-7-2026, we cannot make shots closer than this 
        setTargetVelocity(FLYWHEEL_RPM_MAP.get(dist));
        setTargetHoodPose(HOOD_ROT_MAP.get(dist));

        // Push to hardware immediately if already spinning — keeps tracking live
        if (currentState == ShooterState.READY) {
            commandFlywheelVelocity(targetFlywheelMotorRPM);
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    /**
     * Routes flywheel velocity command to the active control mode.
     */
    private void commandFlywheelVelocity(double rpm) {
            io.setFlywheelVelocity(rpm);
    }

    /**
     * Tuning only: spins flywheel to a fixed RPM with no timeout, no indexer, no hood movement.
     * Use this to characterize kV and kP in isolation — watch FlywheelRPM, FlywheelError,
     * and FlywheelAppliedVolts on the dashboard.
     *
     * Hold the bound button to spin; release to stop.
     *
     * @param rpm Target flywheel RPM
     */
    public Command tuneFlywheelCommand(double rpm) {
        return Commands.startEnd(
            () -> {
                targetFlywheelMotorRPM = rpm;
                commandFlywheelVelocity(rpm);
            },
            () -> {
                io.stopFlywheels();
                targetFlywheelMotorRPM = 0.0;
            },
            this
        ).withName("TuneFlywheelRPM");
    }

    // Use for testing Flywheel RPM only
    public Command setHoodFixed(double rotations) {
        return Commands.startEnd(
            () -> {
                targetHoodPoseRot = rotations;
                io.setHoodPose(rotations);
            },
            () -> {
                io.setHoodPose(Constants.Hood.MIN_POSE);
                targetHoodPoseRot = Constants.Hood.MIN_POSE;
            },
            this
        ).withName("TuneHoodPose");
    }

}
