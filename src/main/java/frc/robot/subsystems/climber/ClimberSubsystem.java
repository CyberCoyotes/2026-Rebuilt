package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/*
 * AUTHOR: @imadinosaur1
 *
 * Purpose: Subsystem for the Climber mechanism
 * Functionality: Extend/retract climber arms
 *
 * @see Constants.Climber for hardware configuration
 */

@SuppressWarnings("unused")

public class ClimberSubsystem extends SubsystemBase {

    // Speed Constants
    private static final double HORNS_UP = 0.5;
    private static final double HORNS_DOWN = -0.5;
    private static final double PARADE_REST = 0.0;

    // Threshold for considering the climber "ready"
    private static final double READY_VELOCITY_THRESHOLD = 3000.0; // RPM

    // Hardware; commented out until motors are assigned
     private final TalonFX climbAMotor;
     private final TalonFX climbBMotor;

     // Constructor
     public ClimberSubsystem() {
        climbAMotor = new TalonFX(Constants.Climber.CLIMB_A_MOTOR_ID);
        climbBMotor = new TalonFX(Constants.Climber.CLIMB_B_MOTOR_ID);
     }

       //Climber methods
      public void setArmVolts(double voltage) { // Set arm to extend give it a variable and declare type
         armMotor.setVoltage(voltage); // Full voltage to extend
      }

      public void setHookVolts(double voltage) { // Set hook to move given variable and declare type
         hookMotor.setVoltage(voltage); // Full voltage to move hook
      }

     //Climber command factory

     // Commands for arm movement
      public Command extendFast() {
         return run(() -> setArmVolts(12));

     }

      public Command extendSlow() {
         return run(() -> setArmVolts(3));
      
     }
 
      public Command retractFast() {
         return run(() -> setArmVolts(-12));

      }

      public Command retractSlow() {
         return run(() -> setArmVolts(-3));

      }

      public Command stopArm() {
         return run(() -> setArmVolts(0));
      }
      
      // Commands for hook movement
       public Command hookUp() {
         return run(() -> setHookVolts(6));
       }

      public Command hookDown() {
         return run(() -> setHookVolts(-6));
      }

      public Command hookStop() {
         return run (() -> setHookVolts(0));
      }
         }// End of ClimberSubsystem class
    