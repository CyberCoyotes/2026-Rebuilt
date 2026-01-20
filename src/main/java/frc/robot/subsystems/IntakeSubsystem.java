package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake2.IntakeIO;
import frc.robot.subsystems.intake2.IntakeIO.IntakeIOInputs;

/**
 * IntakeSubsystem - Main intake subsystem with state machine control.
 *
 * HARDWARE:
 * - 1x TalonFX motor (rotator) for spinning intake wheels
 * - 1x TalonFX motor (slide) for extending/retracting intake
 *
 * STATE MACHINE:
 * - IDLE: Slide retracted, motors off
 * - INTAKING: Slide extended, motors intaking
 * - EJECT: Slide extended, motors reverse
 *
 * USAGE:
 * 1. Call startIntaking() to extend and start intake motors
 * 2. Monitor hasGamePiece() from indexer or other sensors
 * 3. Call retract() to pull intake back in when done
 *
 * @author @Isaak3
 */
public class IntakeSubsystem extends SubsystemBase {

    // ===== State Machine =====
    public enum IntakeState {
        IDLE,       // Slide retracted, motors off
        INTAKING,   // Slide extended, motors running forward
        EJECT       // Slide extended, motors running reverse
    }

    // ===== Hardware Interface =====
    private final IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

    // ===== State =====
    private IntakeState currentState = IntakeState.IDLE;

    // ===== Constants =====
    /** Slide position when fully retracted (rotations) */
    private static final double RETRACTED_POSITION = 0.0;

    /** Slide position when fully extended (rotations) */
    private static final double EXTENDED_POSITION = 10.0;  // TODO: Measure actual extension

    /** Rotator motor speed when intaking (percent output) */
    private static final double INTAKE_SPEED = 0.8;  // TODO: Tune

    /** Rotator motor speed when ejecting (percent output, negative) */
    private static final double EJECT_SPEED = -0.5;  // TODO: Tune

    /** Tolerance for slide position (rotations) */
    private static final double SLIDE_TOLERANCE = 0.5;  // TODO: Tune

    /**
     * Creates a new IntakeSubsystem.
     *
     * @param io Hardware interface (real hardware or simulation)
     */
    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Update state machine (automatic transitions handled here if needed)
        updateStateMachine();

        // Log state
        Logger.recordOutput("Intake/State", currentState.toString());
        Logger.recordOutput("Intake/IsExtended", isExtended());
        Logger.recordOutput("Intake/IsRetracted", isRetracted());

        // Update dashboard
        SmartDashboard.putString("Intake State", currentState.toString());
        SmartDashboard.putNumber("Intake Slide Position", inputs.slidePositionRotations);
        SmartDashboard.putBoolean("Intake Extended", isExtended());
    }

    /**
     * Updates the state machine based on current state and sensor readings.
     * Currently no automatic transitions - all transitions are command-driven.
     */
    private void updateStateMachine() {
        // No automatic state transitions for now
        // All transitions are command-driven via startIntaking(), eject(), retract()
    }

    // ===== State Transitions =====

    /**
     * Transitions to a new state and executes entry actions.
     */
    private void setState(IntakeState newState) {
        if (currentState == newState) {
            return;  // Already in this state
        }

        // Log state transition
        Logger.recordOutput("Intake/StateTransition",
            currentState.toString() + " -> " + newState.toString());

        currentState = newState;

        // Execute entry actions for new state
        switch (newState) {
            case IDLE:
                io.setRotator(0.0);
                io.setSlidePosition(RETRACTED_POSITION);
                break;

            case INTAKING:
                io.setSlidePosition(EXTENDED_POSITION);
                io.setRotator(INTAKE_SPEED);
                break;

            case EJECT:
                io.setSlidePosition(EXTENDED_POSITION);
                io.setRotator(EJECT_SPEED);
                break;
        }
    }

    // ===== Public Command Methods =====

    /**
     * Starts intaking (extends slide, runs motors forward).
     */
    public void startIntaking() {
        setState(IntakeState.INTAKING);
    }

    /**
     * Starts ejecting (extends slide, runs motors reverse).
     */
    public void eject() {
        setState(IntakeState.EJECT);
    }

    /**
     * Stops intake and retracts slide.
     */
    public void retract() {
        setState(IntakeState.IDLE);
    }

    /**
     * Stops all motors immediately without changing slide position.
     * Use this for emergency stops.
     */
    public void stopMotors() {
        io.stop();
    }

    /**
     * Manually sets rotator speed (for testing/tuning).
     *
     * @param percent Motor output from -1.0 to 1.0
     */
    public void setRotatorSpeed(double percent) {
        io.setRotator(percent);
    }

    /**
     * Manually sets slide position (for testing/tuning).
     *
     * @param rotations Target position in rotations
     */
    public void setSlidePosition(double rotations) {
        io.setSlidePosition(rotations);
    }

    /**
     * Zeros the slide encoder at current position.
     * Call this when intake is manually positioned at retracted position.
     */
    public void zeroSlide() {
        io.zeroSlide();
    }

    // ===== Status Queries =====

    /**
     * Returns true if intake is extended.
     */
    public boolean isExtended() {
        return Math.abs(inputs.slidePositionRotations - EXTENDED_POSITION) < SLIDE_TOLERANCE;
    }

    /**
     * Returns true if intake is retracted.
     */
    public boolean isRetracted() {
        return Math.abs(inputs.slidePositionRotations - RETRACTED_POSITION) < SLIDE_TOLERANCE;
    }

    /**
     * Gets current state.
     */
    public IntakeState getState() {
        return currentState;
    }

    /**
     * Gets current slide position in rotations.
     */
    public double getSlidePosition() {
        return inputs.slidePositionRotations;
    }

    /**
     * Gets current rotator velocity in RPS.
     */
    public double getRotatorVelocity() {
        return inputs.rotatorVelocityRPS;
    }

    // ===== Diagnostics =====

    /**
     * Returns true if rotator motor is over-temperature.
     */
    public boolean isOverheating() {
        return inputs.rotatorTempCelsius > 80.0 || inputs.slideTempCelsius > 80.0;
    }

    /**
     * Returns true if rotator current is too high (possible jam).
     */
    public boolean isOverCurrent() {
        return inputs.rotatorCurrentAmps > 40.0;  // TODO: Tune threshold
    }

    /**
     * Returns true if intake might be jammed.
     * Detects high current with low velocity while motors are running.
     */
    public boolean isJammed() {
        boolean motorsRunning = currentState == IntakeState.INTAKING || currentState == IntakeState.EJECT;
        boolean highCurrent = inputs.rotatorCurrentAmps > 30.0;  // TODO: Tune
        boolean lowVelocity = Math.abs(inputs.rotatorVelocityRPS) < 10.0;  // TODO: Tune

        return motorsRunning && highCurrent && lowVelocity;
    }
}
