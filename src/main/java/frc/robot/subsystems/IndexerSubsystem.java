/*
 * AUTHOR: @Joel-Trumpet-67
 */



/*public class Indexer extends SubsystemBase {

  private static final int Indexer_MOTOR_ID_1 = 51;
  private static final int Indexer_MOTOR_ID_2 = 52;
  private static final int Indexer_MOTOR_ID_3 = 53;
  private static final int Indexer_MOTOR_ID_4 = 54;
  private static final int Distance_SENSOR_PORT = 1;

  private final TalonFX IndexerMotor1;
  private final TalonFX IndexerMotor2;
  private final TalonFX IndexerMotor3;
  private final TalonFX IndexerMotor4;
  private final DigitalInput DistanceSensor;

  private boolean isEngaged = false;

  /*
   * The constructor in a Java subsystem is where you set up and configure everything your subsystem needs before it starts operating.
   * It runs exactly once when the robot code starts up (during robotInit()).
   
  public Indexer() { 
    IndexerMotor1 = new TalonFX(Indexer_MOTOR_ID_1);
    IndexerMotor2 = new TalonFX(Indexer_MOTOR_ID_2);
    IndexerMotor3 = new TalonFX(Indexer_MOTOR_ID_3);
    IndexerMotor4 = new TalonFX(Indexer_MOTOR_ID_4);
    DistanceSensor = new DigitalInput(Distance_SENSOR_PORT);
  }

    public void setHopperSpeed(double HopperPower) { 
/* 
      if (isMotivatorFunctional()) {
            IndexerMotor1.set(1); // Why set speeds to 1,2,3,4?
            IndexerMotor2.set(2);
            IndexerMotor3.set(3);
            IndexerMotor4.set(4);
        } else {
           
            //  name doesn't match any of your motors above. e.g. IndexerMotor1
            // IndexerMotor.set(1);
            System.out.println("WARNING: Indexer motivator malfunction!");
        }

    }

    
    public void engage() {}
/* 
      if (isReady() && isMotivatorFunctional()) {
            IndexerMotor1.set(ENGAGE_SPEED);
            isEngaged = true;
        } else {
            System.out.println("WARNING: Indexer not ready for engagement!");
        }
  


    
    public void emergancyStop() {

        // IndexerMotor1.set(IDLE_SPEED);
        isEngaged = false;
        System.out.println("Indexer EMERGENCY STOP ACTIVATED");
    }


    
/*
    public boolean isMotivatorFunctional() {
       return !motivatorSensor.get();

    }
*/

/* 
    public boolean isReady() {
        double currentVelocity = IndexerMotor.getVelocity().getValueAsDouble();
        boolean hasVelocity = currentVelocity >= READY_VELOCITY_THRESHOLD;
        
        return hasVelocity && isMotivatorFunctional();
    }

    
    public boolean isInHyperspace() {
        return isEngaged;
    }
    
/* 
    public double getVelocity() {
        return IndexerMotor.getVelocity().getValueAsDouble();
    }


    @Override
      public void periodic() {

        // if (isEngaged && !isMotivatorFunctional()) {
            // System.out.println("CRITICAL: Motivator failure during hyperspace!");
            // emergencyStop();
        }
        
      
} // End of Class


 */
/* 2 motor 
 * 1 on floor
 * 1 on elevator 
 * sensor for ball to elevator
 */

 package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

 public class IndexerSubsystem {
    
    // ========== CONSTANTS ==========
    private static final int FLOOR_MOTOR_ID = 67;
    private static final int KICK_MOTOR_ID = 67;
    private static final int Sensor = 67;

     // ========== HARDWARE ==========
    private final TalonFX floorMotor;
    private final TalonFX kickMotor;
    private final DigitalInput Sensor;
 

    // Constructor
    public IndexerSubsystem() {

    }
    

 }
