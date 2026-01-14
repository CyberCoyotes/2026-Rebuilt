package frc.robot.training;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SubsystemBase {

  private static final int Indexer_MOTOR_ID_1 = 51;
  private static final int Indexer_MOTOR_ID_2 = 52;
  private static final int Indexer_MOTOR_ID_3 = 53;
  private static final int Indexer_MOTOR_ID_4 = 54;
  private static final int Distance_SENSOR_PORT = 1 

  private final TalonFX IndexerMotor1;
  private final TalonFX IndexerMotor2;
  private final TalonFX IndexerMotor3;
  private final TalonFX IndexerMotor4;
  private final DigitalInput DistanceSensor;

  private boolean isEngaged = false;

  public Indexer() { 
    IndexerMotor = new TalonFX(Indexer_MOTOR_ID);
        motivatorSensor = new DigitalInput(MOTIVATOR_SENSOR_PORT);

    public void charge() { 

      if (isMotivatorFunctional()) {
            IndexerMotor.set(CHARGE_SPEED);
        } else {
           
            IndexerMotor.set(IDLE_SPEED);
            System.out.println("WARNING: Indexer motivator malfunction!");
        }
    }

    public void engage() { 

      if (isReady() && isMotivatorFunctional()) {
            IndexerMotor.set(ENGAGE_SPEED);
            isEngaged = true;
        } else {
            System.out.println("WARNING: Indexer not ready for engagement!");
        }
    }

    public void emergancyStop() {
       IndexerMotor.set(IDLE_SPEED);
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
    
