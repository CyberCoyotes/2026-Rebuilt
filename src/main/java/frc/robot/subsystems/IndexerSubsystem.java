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
    
    /* ToF  sensor */
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


    
    // ========== SENSOR METHODS ==========
    // public boolean isBallAtKick() {
        // return !feederToF(); // Assuming sensor returns false when ball is present
    // }

    // ========== MOTOR METHODS ==========
    public void setFloorMotorSpeed(double speed) {
        floorMotor.set(speed);
    }

    public void setKickMotorSpeed(double speed) {
        feederMotor.set(speed);
    }

 }

