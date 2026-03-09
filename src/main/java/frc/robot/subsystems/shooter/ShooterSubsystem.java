package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

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
 * - IDLE: All motors off, hood at home position
 * - STANDBY: Reserved for future pre-rev use — not active, flywheel command is commented out
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

@SuppressWarnings("unused") // Unused methods are intentional for future use and testing

public class ShooterSubsystem extends SubsystemBase {

    // =========================================================================
    // SHOT PRESETS
    // =========================================================================

    /**
     * Named shot presets that can be selected via POV left/right on the driver controller.
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

    // ==== Hardware Interface ====
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();

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
    private final DoublePublisher  flywheelTempPublisher;
    private final BooleanPublisher flywheelAtRpmPublisher;
    private final StringPublisher  selectedPresetPublisher;

    // ==== State ====
    private ShooterState currentState     = ShooterState.IDLE;
    private ShotPreset   displayPreset    = null; // null = Vision mode (no operator button held)
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
        flywheelTempPublisher        = shooterTable.getDoubleTopic("FlywheelMaxTempCelsius").publish();
        flywheelAtRpmPublisher       = shooterTable.getBooleanTopic("FlywheelAtRPM").publish();
        selectedPresetPublisher       = shooterTable.getStringTopic("SelectedPreset").publish();
    }

    // ==== State Machine Enums ====
    public enum ShooterState {
        IDLE,    // Motors off — only used on explicit stop, not during normal match play
        SPINNING_UP, // Flywheel ramping to target after trigger pull — transitions to READY when at speed
        STANDBY, // Flywheel spinning at STANDBY_RPM; ready to ramp to target on demand
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
        flywheelRpsPublisher.set(inputs.flywheelLeaderMotorRPS);
        flywheelErrorPublisher.set(targetFlywheelMotorRPM - inputs.flywheelLeaderMotorRPM);
        hoodRotationsPublisher.set(inputs.hoodPositionRotations);
        targetHoodRotationsPublisher.set(targetHoodPoseRot);
        hoodErrorPublisher.set(targetHoodPoseRot - inputs.hoodPositionRotations);
        hoodAtPosePublisher.set(isHoodAtPose());
        flywheelVoltsPublisher.set(inputs.flywheelAppliedVolts);
        flywheelTempPublisher.set(inputs.flywheelMaxTempCelsius);
        flywheelAtRpmPublisher.set(isFlywheelAtVelocity());
        selectedPresetPublisher.set(displayPreset != null ? displayPreset.label : "Vision");
    }

    // =====STATE MACHINE=====
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                break;
            case STANDBY:
                break;
            case SPINNING_UP:
                commandFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;
            case READY:
                // Re-issue every cycle so any silent target change (setTargetVelocity, updateFromDistance)
                // takes effect immediately, and so the TalonFX recovers automatically if it
                // drops its setpoint due to a transient fault or CAN dropout.
                commandFlywheelVelocity(targetFlywheelMotorRPM);
                io.setHoodPose(targetHoodPoseRot);
                break;
            case PASS:
                commandFlywheelVelocity(Constants.Shooter.PASS_RPM);
                io.setHoodPose(Constants.Shooter.PASS_HOOD);
                break;
            case EJECT:
                commandFlywheelVelocity(Constants.Shooter.EJECT_RPM);
                break;
            case POPPER:
                commandFlywheelVelocity(Constants.Shooter.POPPER_RPM);
                io.setHoodPose(Constants.Shooter.POPPER_HOOD);
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
                // io.setFlywheelVelocity(STANDBY_RPM); // TODO: Do not use right now **EXPERIMENTAL**
                io.setHoodPose(Constants.Shooter.MIN_HOOD_POSE_ROT);
                break;

            case READY:
                // io.setFlywheelVelocity(targetFlywheelMotorRPM);
                commandFlywheelVelocity(targetFlywheelMotorRPM); // Routes to the active control mode (VelocityVoltage or VelocityTorqueCurrentFOC)
                io.setHoodPose(targetHoodPoseRot);
                break;

            case PASS:
                // io.setFlywheelVelocity(PASS_RPM);
                commandFlywheelVelocity(Constants.Shooter.PASS_RPM); // Routes to the active control mode (VelocityVoltage or VelocityTorqueCurrentFOC)
                io.setHoodPose(Constants.Shooter.PASS_HOOD);
                break;

            case EJECT:
                // io.setFlywheelVelocity(EJECT_RPM);
                commandFlywheelVelocity(Constants.Shooter.EJECT_RPM); // Routes to the active control mode (VelocityVoltage or VelocityTorqueCurrentFOC)
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

    /** Full stop — stops flywheels and returns hood to home. */
    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    /**
     * Resets shooter after a shot. Currently routes to IDLE because STANDBY
     * (pre-rev) is not yet active. Update this when STANDBY spin logic is validated.
     */
    public void returnToStandby() {
        setState(ShooterState.IDLE);
    }

    /**
     * Transitions to SPINNING_UP, ramping flywheel to target and moving hood to target angle.
     */
    public void spinUpToShoot() {
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
        if (Math.abs(getCurrentVelocityRPM()) > Constants.Shooter.EJECT_MAX_ENTRY_RPM) {
            return; // Flywheel spinning too fast to safely reverse — refuse eject
        }
        setState(ShooterState.EJECT);
    }

    // ===== Display Preset (Elastic dashboard sanity check) =====

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

    // ===== Target Setters =====

    /** Sets target flywheel velocity (forward only — clamped to MAX_FLYWHEEL_RPM). Does NOT change state. */
    public void setTargetVelocity(double rpm) {
        targetFlywheelMotorRPM = Math.min(Math.abs(rpm), Constants.Shooter.MAX_FLYWHEEL_RPM);
    }

    /**
     * Sets target hood pose in rotations. Clamped to valid range. Does NOT change state.
     */
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
        double tolerance = Math.abs(targetFlywheelMotorRPM) * Constants.Shooter.FLYWHEEL_TOLERANCE_PERCENT;
        return Math.abs(inputs.flywheelLeaderMotorRPM - targetFlywheelMotorRPM) < tolerance;
    }

    /** Returns true if hood is at target pose within tolerance. */
    public boolean isHoodAtPose() {
        return Math.abs(inputs.hoodPositionRotations - targetHoodPoseRot) < Constants.Shooter.HOOD_POSE_TOLERANCE;
    }

    /** Returns true if total flywheel current is too high. */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0;
    }

    // =========================================================================
    // Commands
    // =========================================================================

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

    public Command spinUp() {
        return Commands.startEnd(
            () -> setState(ShooterState.SPINNING_UP),
            () -> setState(ShooterState.IDLE),
            this).withName("Shooter: SpinUp");
    }
        // =========================================================================
    // Vision Lookup Tables
    // =========================================================================

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
        // ==== Flywheel RPM vs. distance =================================
        // TODO: Replace each value with a measured result (see TUNING.md §4)
        FLYWHEEL_RPM_MAP.put(1.5, 2700.0);
        FLYWHEEL_RPM_MAP.put(3.55, 3200.0); 
        FLYWHEEL_RPM_MAP.put(5.5, 3800.0);

        // ==== Hood position (rotations) vs. distance =================================
        // TODO: Replace each value with a measured result (see TUNING.md §4)
        HOOD_ROT_MAP.put(1.5, 0.00); 
        HOOD_ROT_MAP.put(3.55, 4.30); 
        HOOD_ROT_MAP.put(5.5, 5.50);
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
        double dist = Math.max(1.0, Math.min(8.0, distanceMeters)); // bumped to 8
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

}
