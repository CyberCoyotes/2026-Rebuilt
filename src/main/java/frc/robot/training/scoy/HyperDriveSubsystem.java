package frc.robot.training.scoy;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HyperDrive Subsystem - SIMPLE EXAMPLE
 * 
 * This example demonstrates the "start simple" approach:
 * - Boolean helper methods to track state
 * - Explicit, readable action methods
 * - No state machine enum (yet!)
 * 
 * This is the RIGHT way to start. Only add a state enum when:
 * - You have 3+ booleans that interact
 * - You're getting confused about what combinations are valid
 * - Bugs keep appearing from invalid state combinations
 * 
 * "Traveling through hyperspace ain't like dusting crops, boy!"
 *   - Han Solo
 */
public class HyperDriveSubsystem extends SubsystemBase {
    
    // ========== CONSTANTS ==========
    // Use CAN IDs 50-99 for practice (won't conflict with real robot)
    private static final int HYPERDRIVE_MOTOR_ID = 51;
    private static final int MOTIVATOR_SENSOR_PORT = 1;
    
    // Speed constants - named clearly so anyone can understand
    private static final double CHARGE_SPEED = 0.3;
    private static final double ENGAGE_SPEED = 1.0;
    private static final double IDLE_SPEED = 0.0;
    
    // Threshold for considering the hyperdrive "ready"
    private static final double READY_VELOCITY_THRESHOLD = 4000.0; // RPM
    
    // ========== HARDWARE ==========
    private final TalonFX hyperdriveMotor;
    private final DigitalInput motivatorSensor;

    // ========== STATE TRACKING ==========
    // Simple booleans work great when you only have 1-2 to track
    private boolean isEngaged = false;

    // ========== NETWORKTABLES PUBLISHERS ==========
    // Publish to NetworkTables for Elastic dashboard (NOT SmartDashboard/Shuffleboard)
    private final BooleanPublisher readyPublisher;
    private final BooleanPublisher engagedPublisher;
    private final BooleanPublisher motivatorPublisher;
    private final DoublePublisher velocityPublisher;
    
    // ========== CONSTRUCTOR ==========
    public HyperDriveSubsystem() {
        hyperdriveMotor = new TalonFX(HYPERDRIVE_MOTOR_ID);
        motivatorSensor = new DigitalInput(MOTIVATOR_SENSOR_PORT);

        // Initialize NetworkTables publishers for Elastic dashboard
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable hyperDriveTable = inst.getTable("HyperDrive");

        readyPublisher = hyperDriveTable.getBooleanTopic("Ready").publish();
        engagedPublisher = hyperDriveTable.getBooleanTopic("Engaged").publish();
        motivatorPublisher = hyperDriveTable.getBooleanTopic("Motivator").publish();
        velocityPublisher = hyperDriveTable.getDoubleTopic("Velocity").publish();

        // Configure motor defaults
        // In a real subsystem, you'd configure:
        // - Current limits
        // - Neutral mode (brake/coast)
        // - Inverted or not
    }
    
    // ========== ACTION METHODS ==========
    // These are the "verbs" - what can the hyperdrive DO?
    
    /**
     * Begin charging the hyperdrive for a jump.
     * Must charge before engaging!
     */
    public void charge() {
        // Only charge if motivator is working
        if (isMotivatorFunctional()) {
            hyperdriveMotor.set(CHARGE_SPEED);
        } else {
            // Bad motivator! Can't charge.
            hyperdriveMotor.set(IDLE_SPEED);
            System.out.println("WARNING: Hyperdrive motivator malfunction!");
        }
    }
    
    /**
     * Engage the hyperdrive at full power.
     * "Punch it, Chewie!"
     */
    public void engage() {
        // Safety check: only engage if ready AND motivator works
        if (isReady() && isMotivatorFunctional()) {
            hyperdriveMotor.set(ENGAGE_SPEED);
            isEngaged = true;
        } else {
            System.out.println("WARNING: Hyperdrive not ready for engagement!");
        }
    }
    
    /**
     * Disengage the hyperdrive and return to realspace.
     */
    public void disengage() {
        hyperdriveMotor.set(IDLE_SPEED);
        isEngaged = false;
    }
    
    /**
     * Emergency shutdown - immediate stop regardless of state.
     */
    public void emergencyStop() {
        hyperdriveMotor.set(IDLE_SPEED);
        isEngaged = false;
        System.out.println("HYPERDRIVE EMERGENCY STOP ACTIVATED");
    }
    
    // ========== SENSOR METHODS ==========
    // These are the "questions" - what can we ASK the hyperdrive?
    
    /**
     * Check if the hyperdrive motivator is functional.
     * A bad motivator will prevent hyperspace jumps.
     * 
     * "Look sir, droids!" ... "This R2 unit has a bad motivator."
     * 
     * @return true if motivator is operational
     */
    public boolean isMotivatorFunctional() {
        // DigitalInput returns true when circuit is open (not connected)
        // Motivator sensor is wired to return false when functional
        // So we invert: false from sensor = true (functional)
        return !motivatorSensor.get();
    }
    
    /**
     * Check if the hyperdrive is charged and ready for engagement.
     * Requires sufficient velocity AND functional motivator.
     * 
     * @return true if ready to make the jump to lightspeed
     */
    public boolean isReady() {
        double currentVelocity = hyperdriveMotor.getVelocity().getValueAsDouble();
        boolean hasVelocity = currentVelocity >= READY_VELOCITY_THRESHOLD;
        
        return hasVelocity && isMotivatorFunctional();
    }
    
    /**
     * Check if currently traveling through hyperspace.
     * 
     * @return true if hyperdrive is engaged
     */
    public boolean isInHyperspace() {
        return isEngaged;
    }
    
    /**
     * Get the current hyperdrive velocity for diagnostics.
     * 
     * @return current motor velocity in RPM
     */
    public double getVelocity() {
        return hyperdriveMotor.getVelocity().getValueAsDouble();
    }
    
    // ========== PERIODIC ==========
    
    @Override
    public void periodic() {
        // This runs every 20ms - use it for:
        // - Logging/telemetry
        // - Safety checks
        // - Automatic state updates

        // Example: Auto-disengage if motivator fails mid-jump
        if (isEngaged && !isMotivatorFunctional()) {
            System.out.println("CRITICAL: Motivator failure during hyperspace!");
            emergencyStop();
        }

        // Publish telemetry to NetworkTables for Elastic dashboard
        readyPublisher.set(isReady());
        engagedPublisher.set(isEngaged);
        motivatorPublisher.set(isMotivatorFunctional());
        velocityPublisher.set(getVelocity());
    }
}
