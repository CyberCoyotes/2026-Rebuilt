/*
 * AUTHOR: @Joel-Trumpet-67
 */

/* 2 motor 
 * 1 on floor
 * 1 on elevator 
 * sensor for ball to elevator
 * /**
 * IndexerSubsystem - Manages game piece movement from intake to shooter.
 *
 * HARDWARE:
 * - Floor motor: Moves game pieces along the hopper floor toward the feeder
 * - Kracken x44
 * - Feeder motor: Grabs pieces from hopper and feeds them into the shooter
 * - Kracken x44
 * - ToF sensor: Detects when a game piece is staged and ready to shoot; might not be needed
 *
 * STATE MACHINE:
 * - IDLE: All motors off, waiting for commands
 * - FLOOR_TRANSPORT: Floor motor on, moving piece toward feeder
 * - FEEDING: Feeder motor on, delivering piece to shooter
 * - INDEXING: Both feeder and floor transport motors on, actively feeding piece to shooter
 * - EJECTING: Both motors reversed, clearing jammed pieces
 *
 * TYPICAL FLOW:
 * - Intake picks up game piece and launches it into hopper
 * - ** UPDATE 1/22 Not needed** FLOOR_TRANSPORT: Floor motor moves piece toward feeder
 * - ** UPDATE 1/22 Not needed** ToF sensor detects piece → transition to IDLE (piece staged)
 * - When Shooting Button pressed && Flywheel Velocity Target met → INDEXING (FLOOR_TRANSPORT && FEEDING) into shooter
 * - ** UPDATE 1//22 Not needed** ToF sensor clears → back to IDLE
 */

 package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.Indexer;

@SuppressWarnings("unused")

public class IndexerSubsystem extends SubsystemBase {
    
     // ========== HARDWARE ========== 
     
    private final IndexerIO io;
    private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();

    // private final TalonFX floorMotor; // These are completely correct in a basic subsystem. 
    // private final TalonFX feederMotor;
    // private final TimeOfFlight feederToF;
    
    // ========== CONSTRUCTOR ==========
    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
        // floorMotor = new TalonFX(Constants.Indexer.FLOOR_MOTOR_ID); // In a basic subsystem this is completely correct
        // feederMotor = new TalonFX(Constants.Indexer.FEEDER_MOTOR_ID);
        // feederToF = new TimeOfFlight(Constants.Indexer.FEEDER_TOF_ID);

    }

    // ========== State Machines ==========


    private boolean isFuelDetected;
    
    // ======== Methods ============
    public void updateSensors(IndexerIOTalonFX input) {
        // Update the fuel detection state based on the ToF sensor reading
        // Consider making the sensor reading a method rather than directly accessing
        /*
         *   public Command Indexing(){
                return runOnce(() -> setFloorMotorSpeed(inputs, 10)); // <-- this need to be actual methods   
        }
         */
        isFuelDetected = input.feederToF.getRange() < Constants.Indexer.FUEL_DETECTION_THRESHOLD;
    }

    public boolean isFuelDetected() {
        return isFuelDetected;
    }
    
    // ========== MOTOR METHODS ==========
    public void setFloorMotorSpeed(IndexerIOTalonFX inputs, double speed) {
        inputs.floorMotor.set(speed);
    }

    public void setKickMotorSpeed(IndexerIOTalonFX inputs, double speed) {
        inputs.feederMotor.set(speed);
    }

    //========= COMMANDS ==========
        // public Command Indexing(){
        //     return runOnce(() -> setFloorMotorSpeed(10));
        // }
 }

