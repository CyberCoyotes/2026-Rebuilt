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
import frc.robot.Constants;

/**
 * STATE MACHINE:
 * - IDLE: All motors off, hood at home position (not used during match)
 * - STANDBY: Flywheel spinning at STANDBY_RPM — always active during match
 * - READY: Flywheel and hood at preset targets, ready to shoot
 * - PASS: Passing shot at PASS_RPM, hood at PASS_HOOD
 * - EJECT: Flywheel reverse at EJECT_RPM for clearing jams — velocity-gated
 *
 * SHOT FLOW:
 * 1. Shooter enabled — enters STANDBY, flywheel spins to STANDBY_RPM
 * 2. Driver presses a preset button (A/X/B) — silently sets target RPM and hood position
 * 3. Driver holds shoot trigger — transitions to READY, hood moves, waits until up to speed, feeds
 * 4. Driver releases trigger — returns to STANDBY
 *
 * FAR SHOT FLOW (X + RT):
 * - X silently arms far shot
 * - RT routes to FarShotCommand instead of shootAtCurrentTarget
 * - FarShotCommand continuously updates hood position via updateHoodForDistance()
 *
 * @see Constants.Shooter for hardware configuration
 * @author @Isaak3
 */

@SuppressWarnings("unused") // Unused methods are intentional for future use and testing

public class ShooterSubsystem extends SubsystemBase {

    // =========================================================================
    // SHOT PRESETS
    // =========================================================================

    /**
     * Named shot presets that can be selected via button on the driver controller.
     * The selected preset is set silently and fired on the right trigger.
     * Published to NetworkTables as Shooter/SelectedPreset for Elastic display.
     */
    public enum ShotPreset {
        CLOSE  ("Close",    Constants.Shooter.CLOSE_RPM,  Constants.Shooter.CLOSE_HOOD),
        TOWER  ("Tower",    Constants.Shooter.TOWER_RPM,  Constants.Shooter.TOWER_HOOD),
        TRENCH ("Trench",   Constants.Shooter.TRENCH_RPM, Constants.Shooter.TRENCH_HOOD),
        PASS   ("Pass",     Constants.Shooter.PASS_RPM,   Constants.Shooter.PASS_HOOD),
        FAR    ("Corner",   Constants.Shooter.FAR_RPM,    Constants.Shooter.FAR_HOOD),
        POPPER ("Popper",   Constants.Shooter.POPPER_RPM, Constants.Shooter.POPPER_HOOD);

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

    // ==== Control Mode Toggle ====
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
    private final BooleanPublisher torqueFOCPublisher;


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
                io.setHoodPose(Constants.Shooter.MIN_HOOD_POSE_ROT);
                break;

            case STANDBY:
                commandFlywheelVelocity(Constants.Shooter.STANDBY_RPM);
                io.setHoodPose(Constants.Shooter.MIN_HOOD_POSE_ROT);
                break;

            case READY:
                commandFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;

            case PASS:
                commandFlywheelVelocity(Constants.Shooter.PASS_RPM);
                io.setHoodPose(Constants.Shooter.PASS_HOOD);
                break;

            case EJECT:
                commandFlywheelVelocity(Constants.Shooter.EJECT_RPM);
                io.setHoodPose(Constants.Shooter.MIN_HOOD_POSE_ROT);
                break;

            case POPPER:
                commandFlywheelVelocity(Constants.Shooter.POPPER_RPM);
                io.setHoodPose(Constants.Shooter.POPPER_HOOD);
                break;
        }
    }

    // =========================================================================
    // PUBLIC COMMAND METHODS
    // =========================================================================

    /** Full stop. Not used during normal match play. */
    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    /**
     * Returns shooter to STANDBY — flywheel idles at STANDBY_RPM, hood returns home.
     * Call this after a shot is complete instead of setIdle().
     */
    public void returnToStandby() {
        setState(ShooterState.STANDBY);
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
        if (Math.abs(getCurrentVelocityRPM()) > Constants.Shooter.EJECT_MAX_ENTRY_RPM) {
            return; // Flywheel spinning too fast to safely reverse — refuse eject
        }
        setState(ShooterState.EJECT);
    }

    // ===== Silent Preset Setters =====

    /** Silently sets close shot targets. No motor movement until shoot trigger is pressed. */
    public void setCloseShotPreset() {
        targetFlywheelMotorRPM = Constants.Shooter.CLOSE_RPM;
        targetHoodPoseRot = Constants.Shooter.CLOSE_HOOD;
    }

    /** Silently sets far shot targets. No motor movement until shoot trigger is pressed. */
    public void setFarShotPreset() {
        targetFlywheelMotorRPM = Constants.Shooter.FAR_RPM;
        targetHoodPoseRot = Constants.Shooter.FAR_HOOD;
    }

    /** Silently sets pass shot targets. No motor movement until shoot trigger is pressed. */
    public void setPassShotPreset() {
        targetFlywheelMotorRPM = Constants.Shooter.PASS_RPM;
        targetHoodPoseRot = Constants.Shooter.PASS_HOOD;
    }

    /** Silently sets popper shot targets. No motor movement until intake trigger is pressed. */
    public void setAirPopper() {
        targetFlywheelMotorRPM = Constants.Shooter.POPPER_RPM;
        targetHoodPoseRot = Constants.Shooter.POPPER_HOOD;
    }

    // ===== Target Setters =====

    /** Sets target flywheel velocity (forward only — clamped to MAX_FLYWHEEL_RPM). Does NOT change state. */
    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), Constants.Shooter.MAX_FLYWHEEL_RPM);
    }

    /** Sets target hood pose in rotations. Clamped to valid range. Does NOT change state. */
    public void setTargetHoodPose(double rotations) {
        targetHoodPoseRot = Math.max(Constants.Shooter.MIN_HOOD_POSE_ROT, Math.min(Constants.Shooter.MAX_HOOD_POSE_ROT, rotations));
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
        double tolerance = Math.abs(targetFlywheelMotorRPM) * Constants.Shooter.FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    /** Returns true if hood is at target pose within tolerance. */
    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodPositionRotations - targetHoodPoseRot) < Constants.Shooter.HOOD_POSE_TOLERANCE;
    }

    /** Returns true if total flywheel current is too high. */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0; // TODO: Tune threshold
    }

    // ── Commands ───────────────────────────────────────────────────────────────

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
    // PRESET CYCLING
    // =========================================================================

    /**
     * Arms the currently selected preset silently (RPM + hood).
     * No motor movement until shoot trigger is pressed.
     */
    public void setSelectedPreset() {
        targetFlywheelMotorRPM = selectedPreset.rpm;
        targetHoodPoseRot      = selectedPreset.hood;
    }

    /** Returns the currently selected preset. */
    public ShotPreset getSelectedPreset() {
        return selectedPreset;
    }

    /**
     * Directly selects a specific preset and arms it silently.
     * The selection sticks until another preset is chosen.
     */
    public void selectPreset(ShotPreset preset) {
        selectedPreset = preset;
        setSelectedPreset();
    }

    /**
     * Advances to the next preset in the cycle order.
     * Arms the new preset silently so it's ready when RT is pressed.
     */
    public void cyclePresetForward() {
        selectedPreset = PRESETS[(selectedPreset.ordinal() + 1) % PRESETS.length];
        setSelectedPreset();
    }

    /**
     * Moves to the previous preset in the cycle order.
     * Arms the new preset silently so it's ready when RT is pressed.
     */
    public void cyclePresetBackward() {
        selectedPreset = PRESETS[(selectedPreset.ordinal() - 1 + PRESETS.length) % PRESETS.length];
        setSelectedPreset();
    }

    /** Returns true if the FAR preset is currently selected (used to gate FarShotCommand). */
    public boolean isFarShotSelected() {
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
     */
    private static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HOOD_ROT_MAP     = new InterpolatingDoubleTreeMap();

    static {
        // ── Flywheel RPM vs. distance ──────────────────────────────────────────
        FLYWHEEL_RPM_MAP.put(1.0, 2500.0); // TODO: tune
        FLYWHEEL_RPM_MAP.put(6.0, 4100.0); // TODO: tune  (hw max ~6380 RPM)

        // ── Hood position (rotations) vs. distance ────────────────────────────
        HOOD_ROT_MAP.put(1.0, 0.00); // TODO: tune  (0.0 = fully up / close)
        HOOD_ROT_MAP.put(6.0, 5.00); // TODO: tune  (hw max ~9.14 rot)
    }

    /**
     * Updates shooter targets from the interpolation maps for the given distance.
     * If already in READY state, pushes new targets to hardware immediately.
     *
     * @param distanceMeters Measured distance to hub tag in meters (floor distance)
     */
    public void updateFromDistance(double distanceMeters) {
        double dist = Math.max(1.0, Math.min(8.0, distanceMeters));
        setTargetVelocity(FLYWHEEL_RPM_MAP.get(dist));
        setTargetHoodPose(HOOD_ROT_MAP.get(dist));

        if (currentState == ShooterState.READY) {
            commandFlywheelVelocity(targetFlywheelMotorRPM);
            io.setHoodPose(targetHoodPoseRot);
        }
    }

    // ==== LEGACY / CONVENIENCE SHIMS ====

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

    /**
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