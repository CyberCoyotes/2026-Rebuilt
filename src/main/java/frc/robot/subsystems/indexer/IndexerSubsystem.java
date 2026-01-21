/*
 * AUTHOR: @Joel-Trumpet-67
 */

/* 2 motor 
 * 1 on floor
 * 1 on elevator 
 * sensor for ball to elevator
 */

 package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.Indexer;

@SuppressWarnings("unused")

public class IndexerSubsystem {
    
     // ========== HARDWARE ==========
    private final TalonFX floorMotor;
    private final TalonFX feederMotor;
    private final TimeOfFlight feederToF;
    
    

    // ========== CONSTRUCTOR ==========
    public IndexerSubsystem() {
        floorMotor = new TalonFX(Constants.Indexer.FLOOR_MOTOR_ID);
        feederMotor = new TalonFX(Constants.Indexer.FEEDER_MOTOR_ID);
        feederToF = new TimeOfFlight(Constants.Indexer.FEEDER_TOF_ID);
    }

    // ========== State Machines ==========
    private boolean isFuelDetected = false;
    public void updateSensors() {
        // Update the fuel detection state based on the ToF sensor reading
        isFuelDetected = feederToF.getRange() < Constants.Indexer.FUEL_DETECTION_THRESHOLD;
    }
    public boolean isFuelDetected() {
        return isFuelDetected;
    }
    
    // ========== MOTOR METHODS ==========
    public void setFloorMotorSpeed(double speed) {
        floorMotor.set(speed);
    }

    public void setKickMotorSpeed(double speed) {
        feederMotor.set(speed);
    }

    //========= COMMANDS ==========
    // public class MySubsystemCommands {
        // private Indexer() {}

        public static Command doSomething (IndexerSubsystem indexer){
            return Commands.startEnd(
                () -> Indexer.Index(), // <-- this need to be actual methods
                () -> Indexer.stop()
            );
        }
    // }
 }

