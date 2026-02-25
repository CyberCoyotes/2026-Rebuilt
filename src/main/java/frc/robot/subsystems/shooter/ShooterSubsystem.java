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
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

/**
 * THESE COMMENTS ARE ALL FROM EXPERIMENTAL AND SHOULD NOT DRIVE FUNCTIONALITY OF THE SHOOTER SUBSYSTEM. 
 * STATE MACHINE:
 * - IDLE: All motors off, hood at home position (not used during match)
 * - STANDBY: Flywheel spinning at STANDBY_RPM — always active during match
 * - READY: Flywheel and hood at preset targets, ready to shoot
 * - PASS: Passing shot at PASS_RPM, hood at PASS_HOOD
 * - EJECT: Flywheel reverse at EJECT_RPM for clearing jams — velocity-gated
 *
 * SHOT FLOW:
 * 1. Shooter enabled — spinup() called, flywheel spins to STANDBY_RPM
 * 2. Driver presses a preset button (A/X/B) — silently sets target RPM and hood position
 * 3. Driver holds shoot trigger — transitions to READY, hood moves, waits until up to speed, feeds
 * 4. Driver releases trigger — returns to IDLE (TODO: change to STANDBY once spin logic validated)
 *
 * FAR SHOT FLOW (X + RT): [NOT YET ACTIVE — isFarShotArmed disabled]
 * - X silently arms far shot
 * - RT routes to FarShotCommand instead of shootAtCurrentTarget
 * - FarShotCommand continuously updates hood position via updateHoodForDistance()
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */

@SuppressWarnings("unused") // Unused methods are intentional for future use and testing

public class ShooterSubsystem extends SubsystemBase {

    // == Constants ======
     // --- Flywheel ---
    private static final double MAX_FLYWHEEL_RPM = 6380.0;
    private static final double IDLE_RPM = 0;

    /** TODO tune RPMs for flywheel without excessive current draw
     * Add an end of line comment `Tuned` when each is verified */
    private static final double POPPER_RPM  = 800; // TODO: Tune
    private static final double STANDBY_RPM = 1000; //
    public static final double CLOSE_RPM   = 2750; //
    private static final double TOWER_RPM   = 3200; // TODO: Tune was 3100, 4.42
    private static final double TRENCH_RPM  = 3200; // TODO: Tune
    private static final double FAR_RPM     = 3800; // TODO: Tune was 4000 + 5.5 worked
    private static final double PASS_RPM    = 4000; //

    /** Reverse RPM for jam clearing. Only reached through eject(), which gates on EJECT_MAX_ENTRY_RPM. */
    private static final double EJECT_RPM = -1500;
    /** Maximum forward flywheel RPM at which EJECT is safe to enter. Prevents violent reversal. */
    private static final double EJECT_MAX_ENTRY_RPM = 500.0;

    private static final double FLYWHEEL_TOLERANCE_PERCENT = 0.10; // TODO Consider a tighter tolerance than 10%

     // --- Hood (Kraken rotational positions) ---
    public static final double MIN_HOOD_POSE_ROT  = 0.0;  // Mechanical limit, validate in configs limit
    public static final double MAX_HOOD_POSE_ROT  = 9.14; // Mechanical limit, validate in configs limit
    public static final double HOOD_POSE_TOLERANCE = 0.25; // TODO Tune tolerance based on testing — consider a tighter tolerance than 0.25 rotations

    /** TODO tune Hood rotation position values from Kraken encoder for each shot
     * Consider using WCP Encoder
     * Add an end of line comment `Tuned` when each is verified */
    public static final double CLOSE_HOOD  = 0.00; //
    public static final double POPPER_HOOD  = 8.42; // TODO: Tune
    public static final double TOWER_HOOD  = 4.30; //
    public static final double TRENCH_HOOD = 4.30; // TODO: Tune
    public static final double FAR_HOOD    = 5.50; // TODO: Tune was 4000 + 5.5 worked
    public static final double PASS_HOOD   = 7.00; //

     // --- Testing Increments ---
    public static final double HOOD_TEST_INCREMENT         = 0.2;
    public static final double FLYWHEEL_TEST_INCREMENT_RPM = 100.0;

    // =========================================================================
    // SHOT PRESETS (for cycling via POV left/right)
    // =========================================================================

    /**
     * Named shot presets that can be selected via POV left/right on the driver controller.
     * The selected preset is armed silently and fired on the right trigger.
     * Published to NetworkTables as Shooter/SelectedPreset for Elastic display.
     */
    public enum ShotPreset {
        CLOSE  ("Close",   CLOSE_RPM,  CLOSE_HOOD),
        TOWER  ("Tower",   TOWER_RPM,  TOWER_HOOD),
        TRENCH ("Trench",  TRENCH_RPM, TRENCH_HOOD),
        PASS   ("Pass",    PASS_RPM,   PASS_HOOD),
        FAR    ("Far",     FAR_RPM,    FAR_HOOD),
        POPPER ("Popper", POPPER_RPM, POPPER_HOOD); // TODO: Tune

        public final String label;
        public final double rpm;
        public final double hood;

        ShotPreset(String label, double rpm, double hood) {
            this.label = label;
            this.rpm   = rpm;
            this.hood  = hood;
        }
    }

    private static final ShotPreset[] PRESETS = ShotPreset.values();

    // ==== Hardware Interface ====
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

    // ==== TODO Control Mode Toggle ====
    /**
     * When true, flywheel uses VelocityTorqueCurrentFOC (Slot 1 gains).
     * When false, uses VelocityVoltage with FOC enabled (Slot 0 gains) — default.
     *
     * NOTE: TorqueCurrentFOC requires CAN FD. On RIO CAN this flag should stay false
     * until flywheel motors are moved to CANivore.
     */
    private boolean useTorqueFOC = false;

    // ==== Dashboard Publishers (NetworkTables) ====
    private final NetworkTable shooterTable;
    private final StringPublisher  statePublisher;
    private final BooleanPublisher readyPublisher;
    private final DoublePublisher  flywheelRpmPublisher;
    private final DoublePublisher  targetRpmPublisher;
    private final DoublePublisher  flywheelRpsPublisher;
    private final DoublePublisher  flywheelErrorPublisher;
    private final DoublePublisher  hoodRotationsPublisher;
    private final DoublePublisher  targetHoodRotationsPublisher;
    private final DoublePublisher  hoodErrorPublisher;
    private final BooleanPublisher hoodAtPosePublisher;
    private final DoublePublisher  flywheelVoltsPublisher;
    private final DoublePublisher  throughBorePositionPublisher;
    private final BooleanPublisher throughBoreConnectedPublisher;
    private final StringPublisher  selectedPresetPublisher;
    private final BooleanPublisher torqueFOCPublisher; // TODO Test VelocityTorqueCurrentFOC on flywheel — compare to VelocityVoltage with FOC, see if it improves acceleration or stability. Publish active control mode for visibility on dashboard.


    // ==== State ====
    private ShooterState currentState     = ShooterState.IDLE;
    private ShotPreset   selectedPreset   = ShotPreset.CLOSE;
    private String currentStateString     = ShooterState.IDLE.toString();
    private double targetFlywheelMotorRPM = 0.0;
    private double targetHoodPoseRot      = 0.0;

    // Slow publish divider
    private int periodicCounter = 0;

    // ==== CONSTRUCTOR ====
    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        shooterTable = inst.getTable("Shooter");

        statePublisher               = shooterTable.getStringTopic("State").publish();
        readyPublisher               = shooterTable.getBooleanTopic("IsReady").publish();
        flywheelRpmPublisher         = shooterTable.getDoubleTopic("FlywheelRPM").publish();
        targetRpmPublisher           = shooterTable.getDoubleTopic("TargetFlywheelRPM").publish();
        flywheelRpsPublisher         = shooterTable.getDoubleTopic("FlywheelMotorRPS").publish();
        flywheelErrorPublisher       = shooterTable.getDoubleTopic("FlywheelError").publish();
        hoodRotationsPublisher       = shooterTable.getDoubleTopic("HoodRotations").publish();
        targetHoodRotationsPublisher = shooterTable.getDoubleTopic("TargetHoodRotations").publish();
        hoodErrorPublisher           = shooterTable.getDoubleTopic("HoodError").publish();
        hoodAtPosePublisher          = shooterTable.getBooleanTopic("HoodAtPose").publish();
        flywheelVoltsPublisher       = shooterTable.getDoubleTopic("FlywheelAppliedVolts").publish();
        throughBorePositionPublisher  = shooterTable.getDoubleTopic("ThroughBorePosition").publish();
        throughBoreConnectedPublisher = shooterTable.getBooleanTopic("ThroughBoreConnected").publish();
        selectedPresetPublisher       = shooterTable.getStringTopic("SelectedPreset").publish();
        // TODO Test VelocityTorqueCurrentFOC on flywheel — compare to VelocityVoltage with FOC, see if it improves acceleration or stability. Publish active control mode for visibility on dashboard.
        torqueFOCPublisher = shooterTable.getBooleanTopic("UsingTorqueFOC").publish();

    }

    // ==== State Machine ====
    public enum ShooterState {
        IDLE,    // Motors off — only used on explicit stop, not during normal match play
        STANDBY, // Flywheel spinning at STANDBY_RPM; ready to ramp to target on demand
        READY,   // Flywheel and hood at preset targets, ready to shoot
        PASS,    // Passing shot at PASS_RPM, hood at PASS_HOOD
        EJECT,   // Jam clearing: flywheel at EJECT_RPM (reverse), hood at MIN_POSE — velocity-gated
        POPPER   // Popper mode: flywheel and hood at minimum pose for assisting with fuel loading
    }

    // ==== Periodic ====
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        updateStateMachine();

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
        flywheelRpsPublisher.set(inputs.flywheelLeaderMotorRPS);
        flywheelErrorPublisher.set(targetFlywheelMotorRPM - inputs.flywheelLeaderMotorRPM);
        hoodRotationsPublisher.set(inputs.hoodPositionRotations);
        targetHoodRotationsPublisher.set(targetHoodPoseRot);
        hoodErrorPublisher.set(targetHoodPoseRot - inputs.hoodPositionRotations);
        hoodAtPosePublisher.set(isHoodAtPose());
        flywheelVoltsPublisher.set(inputs.flywheelAppliedVolts);
        throughBorePositionPublisher.set(inputs.hoodThroughBorePositionRotations);
        throughBoreConnectedPublisher.set(inputs.hoodThroughBoreConnected);
        selectedPresetPublisher.set(selectedPreset.label);

        // TODO Test VelocityTorqueCurrentFOC on flywheel — compare to VelocityVoltage with FOC, see if it improves acceleration or stability. Publish active control mode for visibility on dashboard.
        torqueFOCPublisher.set(useTorqueFOC);
    }

    // =====STATE MACHINE=====
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                break;
            case STANDBY:
                break;
            case READY:
                break;
            case PASS:
                break;
            case EJECT:
                break;
            case POPPER:
                break;
        }
    }

    private void setState(ShooterState newState) {
        if (currentState == newState)
            return;

        currentState = newState;
        currentStateString = newState.toString();

        switch (newState) {
            case IDLE:
                io.stopFlywheels();
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case STANDBY:
                // io.setFlywheelVelocity(STANDBY_RPM); // TODO: Do not use right now **EXPERIMENTAL**
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;

            case READY:
                // io.setFlywheelVelocity(targetFlywheelMotorRPM);
                commandFlywheelVelocity(targetFlywheelMotorRPM); // Routes to the active control mode (VelocityVoltage or VelocityTorqueCurrentFOC)
                io.setHoodPose(targetHoodPoseRot);
                break;

            case PASS:
                // io.setFlywheelVelocity(PASS_RPM);
                commandFlywheelVelocity(PASS_RPM); // Routes to the active control mode (VelocityVoltage or VelocityTorqueCurrentFOC)
                io.setHoodPose(PASS_HOOD);
                break;

            case EJECT:
                // io.setFlywheelVelocity(EJECT_RPM);
                commandFlywheelVelocity(EJECT_RPM); // Routes to the active control mode (VelocityVoltage or VelocityTorqueCurrentFOC)
                io.setHoodPose(MIN_HOOD_POSE_ROT);
                break;
            case POPPER:
                commandFlywheelVelocity(POPPER_RPM);
                io.setHoodPose(POPPER_HOOD);
                break;
        }
    }

    // =========================================================================
    // PUBLIC COMMAND METHODS
    // =========================================================================

    /** Full stop. Not used during normal match play — call returnToStandby() after a shot instead. */
    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    /**
     * Called after a shot is complete to reset the shooter.
     * TODO: Do not use right now **EXPERIMENTAL**
     */
    public void returnToStandby() {
        setState(ShooterState.IDLE);
    }

    /**
     * Transitions to READY, ramping flywheel to target and moving hood to target angle.
     */
    public void prepareToShoot() {
        setState(ShooterState.READY);
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
        if (Math.abs(getCurrentVelocityRPM()) > EJECT_MAX_ENTRY_RPM) {
            return; // Flywheel spinning too fast to safely reverse — refuse eject
        }
        setState(ShooterState.EJECT);
    }

    // ===== Silent Preset Setters =====

    /**
     * Silently sets close shot targets. No motor movement until shoot trigger is pressed.
     */
    public void setCloseShotPreset() {
        targetFlywheelMotorRPM = CLOSE_RPM;
        targetHoodPoseRot = CLOSE_HOOD;
    }

    /**
     * Silently sets far shot targets. No motor movement until shoot trigger is pressed.
     */
    public void setFarShotPreset() {
        targetFlywheelMotorRPM = FAR_RPM;
        targetHoodPoseRot = FAR_HOOD;
    }

    /**
     * Silently sets pass shot targets. No motor movement until shoot trigger is pressed.
     */
    public void setPassShotPreset() {
        targetFlywheelMotorRPM = PASS_RPM;
        targetHoodPoseRot = PASS_HOOD;
    }

    /**
     * Silently sets popper shot targets. No motor movement until intake trigger is pressed.
     */
    public void shooterAirPopper() {
        targetFlywheelMotorRPM = POPPER_RPM;
        targetHoodPoseRot = POPPER_HOOD;
    }

    // ===== Target Setters =====

    /** Sets target flywheel velocity (forward only — clamped to MAX_FLYWHEEL_RPM). Does NOT change state. */
    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), MAX_FLYWHEEL_RPM);
    }

    /**
     * Sets target hood pose in rotations. Clamped to valid range. Does NOT change state.
     */
    public void setTargetHoodPose(double rotations) {
        targetHoodPoseRot = Math.max(MIN_HOOD_POSE_ROT, Math.min(MAX_HOOD_POSE_ROT, rotations));
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

    // =========================================================================
    // STATUS QUERIES
    // =========================================================================

    /** Returns true if in READY state with flywheel and hood at targets. */
    public boolean isReady() {
        return currentState == ShooterState.READY && isFlywheelAtVelocity() && isHoodAtPose();
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
        double tolerance = Math.abs(targetFlywheelMotorRPM) * FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    /** Returns true if hood is at target pose within tolerance. */
    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodPositionRotations - targetHoodPoseRot) < HOOD_POSE_TOLERANCE;
    }

    /** Returns true if total flywheel current is too high. */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0; // TODO: Tune threshold
    }

    // ── Commands ───────────────────────────────────────────────────────────────

    /** Spins up flywheel to pre-rev speed. */
    // public Command spinUpCommand() {
    //     return Commands.runOnce(this::spinup, this).withName("SpinUp");
    // }

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

    // =========================================================================
    // PRESET CYCLING (POV left/right in RobotContainer)
    // =========================================================================

    /**
     * Arms the currently selected test preset silently (RPM + hood).
     * No motor movement until shoot trigger is pressed.
     */
    public void armSelectedPreset() {
        targetFlywheelMotorRPM = selectedPreset.rpm;
        targetHoodPoseRot      = selectedPreset.hood;
    }

    /** Returns the currently selected test preset. */
    public ShotPreset getSelectedPreset() {
        return selectedPreset;
    }

    /**
     * Advances to the next preset in the cycle order:
     * Close → Tower → Trench → Pass → Far → Close
     * Arms the new preset silently so it's ready when RT is pressed.
     */
    public void cyclePresetForward() {
        selectedPreset = PRESETS[(selectedPreset.ordinal() + 1) % PRESETS.length];
        armSelectedPreset();
    }

    /**
     * Moves to the previous preset in the cycle order:
     * Close → Far → Pass → Trench → Tower → Close
     * Arms the new preset silently so it's ready when RT is pressed.
     */
    public void cyclePresetBackward() {
        selectedPreset = PRESETS[(selectedPreset.ordinal() - 1 + PRESETS.length) % PRESETS.length];
        armSelectedPreset();
    }

    /** Returns true if the FAR preset is currently selected (used to gate FarShotCommand). */
    public boolean isFarShotArmed() {
        return selectedPreset == ShotPreset.FAR;
    }

    // ==== Vision Lookup Tables ====

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
        // ── Flywheel RPM vs. distance ──────────────────────────────────────────
        // TODO: Replace each value with a measured result (see TUNING.md §4)
        FLYWHEEL_RPM_MAP.put(1.0, 2750.0); // TODO: tune
        FLYWHEEL_RPM_MAP.put(2.0, 2900.0); // TODO: tune
        FLYWHEEL_RPM_MAP.put(3.0, 3100.0); // TODO: tune
        FLYWHEEL_RPM_MAP.put(4.0, 3200.0); // TODO: tune
        FLYWHEEL_RPM_MAP.put(5.0, 3300.0); // TODO: tune
        FLYWHEEL_RPM_MAP.put(6.0, 3400.0); // TODO: tune  (hw max ~6380 RPM)

        // ── Hood position (rotations) vs. distance ────────────────────────────
        // TODO: Replace each value with a measured result (see TUNING.md §4)
        HOOD_ROT_MAP.put(1.0, 0.00); // TODO: tune  (0.0 = fully up / close)
        HOOD_ROT_MAP.put(2.0, 1.50); // TODO: tune
        HOOD_ROT_MAP.put(3.0, 3.00); // TODO: tune
        HOOD_ROT_MAP.put(4.0, 4.30); // TODO: tune
        HOOD_ROT_MAP.put(5.0, 5.50); // TODO: tune
        HOOD_ROT_MAP.put(6.0, 6.00); // TODO: tune  (hw max ~9.14 rot)
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
        double dist = Math.max(1.0, Math.min(6.0, distanceMeters));
        setTargetVelocity(FLYWHEEL_RPM_MAP.get(dist));
        setTargetHoodPose(HOOD_ROT_MAP.get(dist));

        // Push to hardware immediately if already spinning — keeps tracking live
        if (currentState == ShooterState.READY) {
            commandFlywheelVelocity(targetFlywheelMotorRPM);
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    // ==== LEGACY / CONVENIENCE SHIMS ====
    // These combine steps that are intentionally separate in the normal shot flow.
    // Prefer setCloseShotPreset() / setFarShotPreset() + prepareToShoot() at the call site.

    /** Used by spinUpCommand(). Transitions to STANDBY. */
    // public void spinup() {
    //     setState(ShooterState.STANDBY);
    // }

    /** @deprecated Bypasses the silent-preset + shoot-trigger split. Use setCloseShotPreset() instead. */
    @Deprecated
    public void closeShot() {
        setCloseShotPreset();
        prepareToShoot();
    }

    /** @deprecated Bypasses the silent-preset + shoot-trigger split. Use setFarShotPreset() instead. */
    @Deprecated
    public void farShot() {
        setFarShotPreset();
        prepareToShoot();
    }

    /** TODO Test VelocityTorqueCurrentFOC on flywheel — compare to VelocityVoltage with FOC, see if it improves acceleration or stability.
     * Routes flywheel velocity command to the active control mode.
     * Toggle useTorqueFOC to switch between VelocityVoltage and VelocityTorqueCurrentFOC.
     */
    private void commandFlywheelVelocity(double rpm) {
        if (useTorqueFOC) {
            io.setFlywheelVelocityTorqueFOC(rpm);
        } else {
            io.setFlywheelVelocity(rpm);
        }
    }

    public Command toggleControlModeCommand() {
    return Commands.runOnce(() -> {
        useTorqueFOC = !useTorqueFOC;
        System.out.println("[Shooter] Control mode: " + (useTorqueFOC ? "TorqueCurrentFOC (Slot 1)" : "VelocityVoltage (Slot 0)"));
    }, this).withName("ToggleFlywheelControlMode");
}
}
