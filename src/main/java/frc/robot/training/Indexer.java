package frc.robot.training;

/*
 * AUTHOR: @Joel-Trumpet-67
Good things you're doing:

- You're attempting to follow the structure of the HyperDrive example
- You have the right imports and are extending SubsystemBase correctly
- You're thinking about multiple motors for your indexer (which is realistic)
Issues to investigate and fix:
Syntax Errors - Your code won't compile. Look carefully at:

Line 17
Line 28-29: name mismatches (see also lines 19-23)
Line 31: Method definition location - check your curly braces!
Organizational Structure - Compare your file to HyperDrive:

Methods should be OUTSIDE the constructor, not inside it
The constructor should close around line 30, then your methods start
Copy-Paste Awareness - You have leftover references to "HyperDrive" concepts:

Lines 59, 71: "isMotivatorFunctional" and "isInHyperspace" - should these be renamed for an Indexer?
Line 84: Calling emergencyStop() but you defined emergancyStop() (typo on line 52)
Missing Constants - You reference variables that don't exist:

CHARGE_SPEED, IDLE_SPEED, ENGAGE_SPEED, READY_VELOCITY_THRESHOLD
Look at HyperDrive lines 25-36 for how to declare these
Think about your subsystem - You declared 4 motors but your methods only use one called "IndexerMotor" that doesn't exist. Do you need all 4 motors? How should they work together?

Tip: Start by getting the basic structure to compile first, then customize for your indexer's actual behavior.
 */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SubsystemBase {

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

  public Indexer() { 
    IndexerMotor1 = new TalonFX(Indexer_MOTOR_ID_1);
    IndexerMotor2 = new TalonFX(Indexer_MOTOR_ID_2);
    IndexerMotor3 = new TalonFX(Indexer_MOTOR_ID_3);
    IndexerMotor4 = new TalonFX(Indexer_MOTOR_ID_4);
    DistanceSensor = new DigitalInput(Distance_SENSOR_PORT);

    public void go() { 

      if (isMotivatorFunctional()) {
            IndexerMotor1.set(1);
            IndexerMotor2.set(2);
            IndexerMotor3.set(3);
            IndexerMotor4.set(4);
        } else {
           
            IndexerMotor.set(1);
            System.out.println("WARNING: Indexer motivator malfunction!");
        }
    }

    public void engage() {}

      if (isReady() && isMotivatorFunctional()) {
            IndexerMotor1.set(ENGAGE_SPEED);
            isEngaged = true;
        } else {
            System.out.println("WARNING: Indexer not ready for engagement!");
        }
    
    
    public void emergancyStop() {
       IndexerMotor1.set(IDLE_SPEED);
        isEngaged = false;
        System.out.println("Indexer EMERGENCY STOP ACTIVATED");
    }


    public boolean isMottivatorFunctional() {
      return !motivatorSensor.get();

    }

     public boolean isReady() {
        double currentVelocity = IndexerMotor.getVelocity().getValueAsDouble();
        boolean hasVelocity = currentVelocity >= READY_VELOCITY_THRESHOLD;
        
        return hasVelocity && isMotivatorFunctional();
    }
  
    public boolean isInHyperspace() {
        return isEngaged;
    }
    
    public double getVelocity() {
        return IndexerMotor.getVelocity().getValueAsDouble();
    }

    @Override
      public void periodic() {

        if (isEngaged && !isMotivatorFunctional()) {
            System.out.println("CRITICAL: Motivator failure during hyperspace!");
            emergencyStop();
        }
        
      
        }
    }
}
