/*
 * AUTHOR: @Joel-Trumpet-67
 */

 
/* Motors
 * -- Floor motor (1), a Kraken X60 that moves balls along the floor of the hopper towards Feeder 
 * -- Feeder Motor (1), a Kraken X60 that grabs the ball from the hooper floor area and moves it into the shooter
 * Sensors
 * -- Playing With Fusion Time-of-Flight (ToF) to make sure a ball doesn't enter the shooter prematurely 
 * Floor motor should be running whenever the intake is running to move balls towards the feeder
 * Floor motor should be running whenever the feeder is running to ensure balls are being supplied to the feeder
 */

/*
Here are some ideas for an Indexer state machine:
Indexer
enum IndexerState {
IDLE, // Everything off
FLOOR_TRANSPORT,// Floor on, feeder off
FEEDING, // Floor + Feeder on
EJECTING // Floor + Feeder reverse
}

// Sensor status as a separate concept
public boolean isGamePieceStaged() {
return tofSensor.getRange() < THRESHOLD;
}

// In your indexer periodic/state machine logic:
if (currentState == FLOOR_TRANSPORT && isGamePieceStaged()) {
setState(IDLE); // Piece reached feeder, stop motors
}

// When shooter is ready:
if (currentState == IDLE && isGamePieceStaged() && shooterReady) {
setState(FEEDING); // Send piece to shooter
}
*/
 package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

@SuppressWarnings("unused")

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

