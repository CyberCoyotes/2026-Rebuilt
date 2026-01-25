/*
 * AUTHOR: @Joel-Trumpet-67
 */

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * IndexerSubsystemBasic - A simplified indexer implementation for training purposes.
 *
 * RECOMMENDED FOR: Newer programmers learning FRC robot programming.
 *
 * This class demonstrates the fundamentals of subsystem design:
 * - Direct hardware instantiation (TalonFX motors, TimeOfFlight sensors)
 * - Simple motor control methods
 * - Basic sensor reading
 *
 * WHY KEEP THIS CLASS:
 * - Easier to understand than the IO pattern (fewer files, less abstraction)
 * - Great for learning how hardware interfaces work
 * - Good stepping stone before learning IndexerIO/IndexerIOHardware pattern
 *
 * HARDWARE:
 * - 2x TalonFX motors (conveyor on floor, indexer for kicking to shooter)
 * - 4x TimeOfFlight sensors (1 indexer, 3 hopper positions A/B/C)
 *
 * WHEN TO GRADUATE TO IndexerSubsystem:
 * Once comfortable with this class, students should learn the IO pattern used
 * in IndexerSubsystem + IndexerIOHardware. The IO pattern enables:
 * - Simulation testing without hardware
 * - AdvantageKit log replay
 * - Cleaner separation of concerns
 *
 * @see IndexerSubsystem for the production implementation using the IO pattern
 * @see IndexerIOHardware for how hardware is abstracted in the IO pattern
 */
@SuppressWarnings("unused")
public class IndexerSubsystemBasic extends SubsystemBase {
    
     // ========== HARDWARE ==========
     /* 
     * Declare private, final instance fields on the IndexerSubsystem class. 
     * private restricts access to the class, 
     * and final means each reference must be assigned exactly once (typically in the constructor)
     * and cannot be reassigned later
     */
    private final TalonFX conveyorMotor;
    private final TalonFX indexerMotor;
    private final TimeOfFlight indexerToF;
    private final TimeOfFlight hopperAToF;
    private final TimeOfFlight hopperBToF;
    private final TimeOfFlight hopperCToF;    
    
    private double FUEL_DETECTION_THRESHOLD = 5; // TODO determine, and check units CM or IN
 


    // ========== CONSTRUCTOR ==========
    public IndexerSubsystemBasic() {
        conveyorMotor = new TalonFX(Constants.Indexer.CONVEYOR_MOTOR_ID);
        indexerMotor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID);
        indexerToF = new TimeOfFlight(Constants.Indexer.INDEXER_TOF_ID);
        hopperAToF = new TimeOfFlight(Constants.Indexer.HOPPER_TOP_A_TOF_ID);
        hopperBToF = new TimeOfFlight(Constants.Indexer.HOPPER_TOP_B_TOF_ID);
        hopperCToF = new TimeOfFlight(Constants.Indexer.HOPPER_TOP_C_TOF_ID);
        
    }

    // ========== State Machines ==========
    private boolean isFuelDetected = false;
    public void updateSensors() {
        // Update the fuel detection state based on the ToF sensor reading
        isFuelDetected = indexerToF.getRange() < FUEL_DETECTION_THRESHOLD;
    }
    public boolean isFuelDetected() {
        return isFuelDetected;
    }
    
    // ========== MOTOR METHODS ==========
    public void setFloorMotorSpeed(double speed) {
        conveyorMotor.set(speed);
    }

    public void setKickMotorSpeed(double speed) {
        indexerMotor.set(speed);
    }

 }

