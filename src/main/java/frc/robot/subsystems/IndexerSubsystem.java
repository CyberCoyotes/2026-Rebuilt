/*
 * AUTHOR: @Joel-Trumpet-67
 */

/* 2 motor 
 * 1 on floor
 * 1 on elevator 
 * sensor for ball to elevator
 */

 package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

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

 }

