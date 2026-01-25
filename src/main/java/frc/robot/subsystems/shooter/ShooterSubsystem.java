package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

/**
 * ShooterSubsystem - Main shooter subsystem with state machine control.
 *
 * HARDWARE:
 * - 3x Falcon 500 motors (main flywheels) for shooting game pieces
 * - 1x TalonFX motor (hood) for angle adjustment
 * - 1x Kraken X60 motor (counter-wheel) for optional backspin
 *
 * STATE MACHINE:
 * - IDLE: All motors off, hood at home position
 * - SPINUP: Flywheel accelerating to target, hood moving to target angle
 * - READY: Both flywheel and hood at target, ready to shoot
 * - EJECT: Flywheel reverse for clearing jams
 *
 * USAGE:
 * 1. Call setTargetVelocity() and setTargetHoodAngle() to set desired shooting parameters
 * 2. Call spinup() to start motors
 * 3. Wait for isReady() to return true
 * 4. Signal indexer to feed game piece
 * 5. Call idle() when done
 *
 * INTEGRATION WITH VISION:
 * - Use updateFromDistance() to automatically calculate velocity/angle from target distance
 * - Vision subsystem provides distance → shooter calculates ballistics
 *
 * @author @Isaak3
 */
public class ShooterSubsystem extends SubsystemBase {

    // ===== State Machine =====
    public enum ShooterState {
        IDLE,      // Flywheel off, hood at home
        SPINUP,    // Flywheel + hood not IDLE and not at READY
        READY,     // Both at target, ready to shoot
        EJECT      // Flywheel reverse (clearing jams)
    }

    // ===== Hardware Interface =====
    private final ShooterIO io;
    private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

    // ===== State =====
    private ShooterState currentState = ShooterState.IDLE;
    private double targetFlywheelRPM = 0.0;
    private double targetHoodAngle = HOME_ANGLE;

    // ===== Constants =====
    /** Hood angle when at home/idle position (degrees) */
    private static final double HOME_ANGLE = 20.0;  // TODO: Measure actual home angle

    /** Minimum hood angle (degrees) */
    private static final double MIN_HOOD_ANGLE = 15.0;  // TODO: Measure actual range

    /** Maximum hood angle (degrees) */
    private static final double MAX_HOOD_ANGLE = 60.0;  // TODO: Measure actual range

    /** Flywheel velocity tolerance (RPM) */
    private static final double VELOCITY_TOLERANCE_RPM = 50.0;  // TODO: Tune

    /** Hood angle tolerance (degrees) */
    private static final double HOOD_TOLERANCE_DEGREES = 1.0;  // TODO: Tune

    /** Eject velocity (negative = reverse) */
    private static final double EJECT_RPM = -500.0;

    // ===== Shooting Presets =====
    /** Close shot velocity (RPM) */
    public static final double CLOSE_SHOT_RPM = 3000.0;  // TODO: Tune
    public static final double CLOSE_SHOT_ANGLE = 25.0;  // TODO: Tune

    /** Far shot velocity (RPM) */
    public static final double FAR_SHOT_RPM = 5000.0;  // TODO: Tune
    public static final double FAR_SHOT_ANGLE = 45.0;  // TODO: Tune

    /** Pass velocity (RPM) */
    public static final double PASS_RPM = 2000.0;  // TODO: Tune
    public static final double PASS_ANGLE = 30.0;  // TODO: Tune

    /**
     * Creates a new ShooterSubsystem.
     *
     * @param io Hardware interface (real hardware or simulation)
     */
    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Update state machine
        updateStateMachine();

        // Log state and targets
        Logger.recordOutput("Shooter/State", currentState.toString());
        Logger.recordOutput("Shooter/TargetFlywheelRPM", targetFlywheelRPM);
        Logger.recordOutput("Shooter/TargetHoodAngle", targetHoodAngle);
        Logger.recordOutput("Shooter/IsReady", isReady());
        Logger.recordOutput("Shooter/FlywheelError", getFlywheelError());
        Logger.recordOutput("Shooter/HoodError", getHoodError());

        // Update dashboard
        SmartDashboard.putString("Shooter State", currentState.toString());
        SmartDashboard.putBoolean("Shooter Ready", isReady());
        SmartDashboard.putNumber("Shooter RPM", inputs.flywheelVelocityRPM);
        SmartDashboard.putNumber("Shooter Target RPM", targetFlywheelRPM);
        SmartDashboard.putNumber("Hood Angle", inputs.hoodAngleDegrees);
        SmartDashboard.putNumber("Hood Target", targetHoodAngle);
    }

    /**
     * Updates the state machine based on current state and sensor readings.
     */
    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                // Stay idle - nothing to do
                break;

            case SPINUP:
                // Check if both flywheel and hood have reached targets
                if (isFlywheelAtVelocity() && isHoodAtTarget()) {
                    setState(ShooterState.READY);
                }
                break;

            case READY:
                // Stay ready - wait for command to idle or eject
                // Check if we've lost velocity/position (shouldn't happen but defensive)
                if (!isFlywheelAtVelocity() || !isHoodAtTarget()) {
                    setState(ShooterState.SPINUP);
                }
                break;

            case EJECT:
                // Stay ejecting - manual transition out
                break;
        }
    }

    // ===== State Transitions =====

    /**
     * Transitions to a new state and executes entry actions.
     */
    private void setState(ShooterState newState) {
        if (currentState == newState) {
            return;  // Already in this state
        }

        // Log state transition
        Logger.recordOutput("Shooter/StateTransition",
            currentState.toString() + " -> " + newState.toString());

        currentState = newState;

        // Execute entry actions for new state
        switch (newState) {
            case IDLE:
                io.stop();
                targetFlywheelRPM = 0.0;
                targetHoodAngle = HOME_ANGLE;
                io.setHoodAngle(HOME_ANGLE);
                break;

            case SPINUP:
                // Targets already set by setTargetVelocity/setTargetHoodAngle
                io.setFlywheelVelocity(targetFlywheelRPM);
                io.setHoodAngle(targetHoodAngle);
                break;

            case READY:
                // Continue running at target velocity
                // (Already set from SPINUP)
                break;

            case EJECT:
                io.setFlywheelVelocity(EJECT_RPM);
                // Hood stays at current position during eject
                break;
        }
    }

    // ===== Public Command Methods =====

    /**
     * Sets the shooter to idle state (motors off, hood home).
     */
    public void setIdle() {
        setState(ShooterState.IDLE);
    }

    /**
     * Starts spinning up to the current target velocity and hood angle.
     * Call setTargetVelocity/setTargetHoodAngle first to set targets.
     */
    public void spinup() {
        if (currentState == ShooterState.IDLE) {
            setState(ShooterState.SPINUP);
        }
    }

    /**
     * Starts ejecting (reverse flywheel) to clear jams.
     */
    public void eject() {
        setState(ShooterState.EJECT);
    }

    /**
     * Stops ejecting and returns to idle.
     */
    public void stopEject() {
        setState(ShooterState.IDLE);
    }

    // ===== Target Setters =====

    /**
     * Sets the target flywheel velocity.
     * Does not automatically start spinning - call spinup() to start.
     *
     * @param rpm Target velocity in RPM
     */
    public void setTargetVelocity(double rpm) {
        this.targetFlywheelRPM = Math.max(0, rpm);  // Clamp to non-negative

        // If already spinning, update velocity immediately
        if (currentState == ShooterState.SPINUP || currentState == ShooterState.READY) {
            io.setFlywheelVelocity(targetFlywheelRPM);
        }
    }

    /**
     * Sets the target hood angle.
     * Does not automatically move hood - call spinup() to start.
     *
     * @param degrees Target angle in degrees
     */
    public void setTargetHoodAngle(double degrees) {
        this.targetHoodAngle = Math.max(MIN_HOOD_ANGLE,
                                        Math.min(MAX_HOOD_ANGLE, degrees));  // Clamp to range

        // If already moving, update angle immediately
        if (currentState == ShooterState.SPINUP || currentState == ShooterState.READY) {
            io.setHoodAngle(targetHoodAngle);
        }
    }

    /**
     * Sets counter-wheel velocity for backspin.
     * Independent of state machine.
     *
     * @param rpm Counter-wheel velocity in RPM
     */
    public void setCounterWheelVelocity(double rpm) {
        io.setCounterWheelVelocity(rpm);
    }

    // ===== Preset Shots =====

    /**
     * Configures shooter for a close shot and starts spinning up.
     */
    public void closeShot() {
        setTargetVelocity(CLOSE_SHOT_RPM);
        setTargetHoodAngle(CLOSE_SHOT_ANGLE);
        spinup();
    }

    /**
     * Configures shooter for a far shot and starts spinning up.
     */
    public void farShot() {
        setTargetVelocity(FAR_SHOT_RPM);
        setTargetHoodAngle(FAR_SHOT_ANGLE);
        spinup();
    }

    /**
     * Configures shooter for passing and starts spinning up.
     */
    public void pass() {
        setTargetVelocity(PASS_RPM);
        setTargetHoodAngle(PASS_ANGLE);
        spinup();
    }

    // ===== Vision Integration =====

    /**
     * Updates shooter targets based on distance to target.
     * Uses ballistic calculations to determine velocity and angle.
     *
     * @param distanceMeters Distance to target in meters
     */
    public void updateFromDistance(double distanceMeters) {
        // TODO: Implement ballistic calculations
        // For now, use simple linear interpolation between close and far shots

        // Clamp distance to reasonable range
        double distance = Math.max(1.0, Math.min(5.0, distanceMeters));

        // Linear interpolation: distance 1m → close shot, 5m → far shot
        double t = (distance - 1.0) / (5.0 - 1.0);  // 0.0 to 1.0

        double velocity = CLOSE_SHOT_RPM + t * (FAR_SHOT_RPM - CLOSE_SHOT_RPM);
        double angle = CLOSE_SHOT_ANGLE + t * (FAR_SHOT_ANGLE - CLOSE_SHOT_ANGLE);

        setTargetVelocity(velocity);
        setTargetHoodAngle(angle);

        Logger.recordOutput("Shooter/VisionDistance", distanceMeters);
        Logger.recordOutput("Shooter/CalculatedRPM", velocity);
        Logger.recordOutput("Shooter/CalculatedAngle", angle);
    }

    // ===== Status Queries =====

    /**
     * Returns true if shooter is ready to shoot (both flywheel and hood at targets).
     */
    public boolean isReady() {
        return currentState == ShooterState.READY;
    }

    /**
     * Returns true if flywheel is at target velocity (within tolerance).
     */
    public boolean isFlywheelAtVelocity() {
        return Math.abs(inputs.flywheelVelocityRPM - targetFlywheelRPM) < VELOCITY_TOLERANCE_RPM;
    }

    /**
     * Returns true if hood is at target angle (within tolerance).
     */
    public boolean isHoodAtTarget() {
        return Math.abs(inputs.hoodAngleDegrees - targetHoodAngle) < HOOD_TOLERANCE_DEGREES;
    }

    /**
     * Gets current flywheel velocity error (target - actual).
     */
    public double getFlywheelError() {
        return targetFlywheelRPM - inputs.flywheelVelocityRPM;
    }

    /**
     * Gets current hood angle error (target - actual).
     */
    public double getHoodError() {
        return targetHoodAngle - inputs.hoodAngleDegrees;
    }

    /**
     * Gets current state.
     */
    public ShooterState getState() {
        return currentState;
    }

    /**
     * Gets current flywheel velocity.
     */
    public double getCurrentVelocityRPM() {
        return inputs.flywheelVelocityRPM;
    }

    /**
     * Gets current hood angle.
     */
    public double getCurrentHoodAngle() {
        return inputs.hoodAngleDegrees;
    }

    /**
     * Gets target flywheel velocity.
     */
    public double getTargetVelocityRPM() {
        return targetFlywheelRPM;
    }

    /**
     * Gets target hood angle.
     */
    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    // ===== Diagnostics =====

    /**
     * Returns true if any flywheel motor is over-temperature.
     */
    public boolean isOverheating() {
        return inputs.flywheelTempCelsius > 80.0;  // TODO: Tune threshold
    }

    /**
     * Returns true if total flywheel current is too high.
     */
    public boolean isOverCurrent() {
        return inputs.flywheelCurrentAmps > 150.0;  // TODO: Tune threshold (sum of 3 motors)
    }

    /**
     * Gets maximum difference between individual flywheel velocities.
     * Useful for detecting if followers are out of sync.
     */
    public double getFlywheelVelocitySpread() {
        double max = Math.max(Math.max(inputs.flywheelAVelocityRPM,
                                       inputs.flywheelBVelocityRPM),
                                       inputs.flywheelCVelocityRPM);
        double min = Math.min(Math.min(inputs.flywheelAVelocityRPM,
                                       inputs.flywheelBVelocityRPM),
                                       inputs.flywheelCVelocityRPM);
        return max - min;
    }
}
