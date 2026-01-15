package frc.robot.training.scoy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * LightsaberHilt Subsystem - STATE MACHINE EXAMPLE
 * 
 * This example demonstrates WHEN and WHY to use a state machine:
 * 
 * Why not just booleans?
 * ----------------------
 * Imagine tracking this with booleans:
 *   - isExtending, isRetracting, isExtended, isRetracted, isBlocked
 * 
 * What if isExtending AND isRetracted are both true? That's impossible,
 * but booleans don't prevent it. With an enum, you're ALWAYS in exactly
 * ONE state. No impossible combinations.
 * 
 * When to upgrade from booleans to enum:
 * - 3+ booleans tracking related state
 * - You're writing code to prevent "impossible" combinations
 * - State transitions have complex rules
 * - You keep finding bugs from conflicting states
 * 
 * "This is the weapon of a Jedi Knight. Not as clumsy or random as a blaster."
 *   - Obi-Wan Kenobi
 */
public class LightsaberHiltSubsytem extends SubsystemBase {
    
    // ========== STATE ENUM ==========
    // All possible states in one place. The saber is ALWAYS in exactly one of these.
    public enum State {
        /** Blade fully retracted, safe to clip to belt */
        RETRACTED,
        /** Blade extending outward */
        EXTENDING,
        /** Blade fully extended, ready for combat */
        EXTENDED,
        /** Blade retracting inward */
        RETRACTING,
        /** Blade locked due to contact/parry - temporary state */
        BLOCKED
    }
    
    // ========== CONSTANTS ==========
    private static final int BLADE_MOTOR_ID = 52;
    private static final int CONTACT_SENSOR_PORT = 2;
    
    private static final double EXTEND_SPEED = 0.8;
    private static final double RETRACT_SPEED = -0.6;
    private static final double HOLD_SPEED = 0.05;  // Small current to hold position
    private static final double IDLE_SPEED = 0.0;
    
    // Position thresholds (in rotations)
    private static final double FULLY_RETRACTED_POSITION = 0.0;
    private static final double FULLY_EXTENDED_POSITION = 10.0;
    private static final double POSITION_TOLERANCE = 0.2;
    
    // Blocked state timeout (seconds)
    private static final double BLOCKED_TIMEOUT_SECONDS = 0.5;
    
    // ========== HARDWARE ==========
    private final TalonFX bladeMotor;
    private final DigitalInput contactSensor;
    
    // ========== STATE ==========
    private State currentState = State.RETRACTED;
    private State desiredState = State.RETRACTED;
    private double blockedTimestamp = 0.0;
    
    // ========== CONSTRUCTOR ==========
    public LightsaberHiltSubsytem() {
        bladeMotor = new TalonFX(BLADE_MOTOR_ID);
        contactSensor = new DigitalInput(CONTACT_SENSOR_PORT);
        
        // Reset encoder position - blade starts retracted
        bladeMotor.setPosition(0);
    }
    
    // ========== STATE REQUEST METHODS ==========
    // External code REQUESTS states. The state machine decides if/when to transition.
    
    /**
     * Request to ignite the lightsaber (extend blade).
     * Will only work from RETRACTED state.
     */
    public void requestIgnite() {
        if (currentState == State.RETRACTED) {
            desiredState = State.EXTENDED;
        }
    }
    
    /**
     * Request to extinguish the lightsaber (retract blade).
     * Can be requested from any state.
     */
    public void requestExtinguish() {
        desiredState = State.RETRACTED;
    }
    
    // ========== SENSOR METHODS ==========
    
    /** Get current blade position in rotations */
    public double getBladePosition() {
        return bladeMotor.getPosition().getValueAsDouble();
    }
    
    /** Check if blade has made contact (parry detected) */
    public boolean isContactDetected() {
        return !contactSensor.get();  // Sensor returns false when triggered
    }
    
    /** Check if blade is fully retracted */
    public boolean isFullyRetracted() {
        return Math.abs(getBladePosition() - FULLY_RETRACTED_POSITION) < POSITION_TOLERANCE;
    }
    
    /** Check if blade is fully extended */
    public boolean isFullyExtended() {
        return Math.abs(getBladePosition() - FULLY_EXTENDED_POSITION) < POSITION_TOLERANCE;
    }
    
    // ========== STATE QUERY METHODS ==========
    // Let external code ask about state without modifying it
    
    /** Get the current state */
    public State getCurrentState() {
        return currentState;
    }
    
    /** Check if saber is ready for combat (fully extended) */
    public boolean isReadyForCombat() {
        return currentState == State.EXTENDED;
    }
    
    /** Check if saber is safe to holster (fully retracted) */
    public boolean isSafeToHolster() {
        return currentState == State.RETRACTED;
    }
    
    // ========== PERIODIC - THE STATE MACHINE HEART ==========
    
    @Override
    public void periodic() {
        // ===== STEP 1: Handle sensor-driven state changes =====
        handleSensorTransitions();
        
        // ===== STEP 2: Handle desired state transitions =====
        handleDesiredStateTransitions();
        
        // ===== STEP 3: Execute behavior for current state =====
        executeCurrentStateBehavior();
        
        // ===== STEP 4: Logging =====
        logState();
    }
    
    /**
     * Check sensors and force state transitions when needed.
     * These transitions happen regardless of desired state.
     */
    private void handleSensorTransitions() {
        // Contact detected while extended = enter BLOCKED state
        if (currentState == State.EXTENDED && isContactDetected()) {
            currentState = State.BLOCKED;
            blockedTimestamp = DriverStation.getMatchTime();
            System.out.println("LIGHTSABER: Contact! Blade blocked.");
        }
        
        // Blocked timeout expired = return to EXTENDED
        if (currentState == State.BLOCKED) {
            double timeInBlocked = DriverStation.getMatchTime() - blockedTimestamp;
            if (timeInBlocked >= BLOCKED_TIMEOUT_SECONDS || !isContactDetected()) {
                currentState = State.EXTENDED;
                System.out.println("LIGHTSABER: Block released, returning to ready.");
            }
        }
        
        // Extending and reached full extension = now EXTENDED
        if (currentState == State.EXTENDING && isFullyExtended()) {
            currentState = State.EXTENDED;
            System.out.println("LIGHTSABER: Blade fully extended. Ready for combat.");
        }
        
        // Retracting and reached full retraction = now RETRACTED
        if (currentState == State.RETRACTING && isFullyRetracted()) {
            currentState = State.RETRACTED;
            System.out.println("LIGHTSABER: Blade retracted. Safe to holster.");
        }
    }
    
    /**
     * Check if we should start moving toward desired state.
     */
    private void handleDesiredStateTransitions() {
        // Want to be extended, but currently retracted = start extending
        if (desiredState == State.EXTENDED && currentState == State.RETRACTED) {
            currentState = State.EXTENDING;
            System.out.println("LIGHTSABER: Igniting blade...");
        }
        
        // Want to be retracted from extended = start retracting
        if (desiredState == State.RETRACTED && currentState == State.EXTENDED) {
            currentState = State.RETRACTING;
            System.out.println("LIGHTSABER: Extinguishing blade...");
        }
        
        // Want to be retracted while extending = start retracting
        if (desiredState == State.RETRACTED && currentState == State.EXTENDING) {
            currentState = State.RETRACTING;
            System.out.println("LIGHTSABER: Aborting ignition, retracting...");
        }
        
        // Want to be retracted while blocked = start retracting
        if (desiredState == State.RETRACTED && currentState == State.BLOCKED) {
            currentState = State.RETRACTING;
            System.out.println("LIGHTSABER: Emergency retract from block!");
        }
    }
    
    /**
     * Run the motor at the appropriate speed for current state.
     * This is the "output" of the state machine.
     */
    private void executeCurrentStateBehavior() {
        switch (currentState) {
            case RETRACTED:
                // Fully retracted - motor off
                bladeMotor.set(IDLE_SPEED);
                break;
                
            case EXTENDING:
                // Moving outward
                bladeMotor.set(EXTEND_SPEED);
                break;
                
            case EXTENDED:
                // Fully extended - small hold current for rigidity
                bladeMotor.set(HOLD_SPEED);
                break;
                
            case RETRACTING:
                // Moving inward
                bladeMotor.set(RETRACT_SPEED);
                break;
                
            case BLOCKED:
                // Contact detected - hold position firmly
                bladeMotor.set(HOLD_SPEED);
                break;
        }
    }
    
    /**
     * Log state for debugging.
     */
    private void logState() {
        // Future: Add SmartDashboard logging
        // SmartDashboard.putString("Lightsaber/State", currentState.name());
        // SmartDashboard.putString("Lightsaber/Desired", desiredState.name());
        // SmartDashboard.putNumber("Lightsaber/Position", getBladePosition());
        // SmartDashboard.putBoolean("Lightsaber/Contact", isContactDetected());
    }
}
