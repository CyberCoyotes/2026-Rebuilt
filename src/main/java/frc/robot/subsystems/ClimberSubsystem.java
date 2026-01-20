package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/* 
 *  
 * AUTHOR: @imadinosaur1
 *
 * Purpose: Subsystem for the Climber mechanism 
 * Hardware: 2 motors; Kraken 60x for extension, Minion for hook movement
 * Functionality: Extend/retract climber arms, move hooks up/down
 * 
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
     private final TalonFX armMotor;
     private final TalonFX hookMotor;

     // Constructor
     public ClimberSubsystem() {
        armMotor = new TalonFX(Constants.Climber.CLIMB_EXTENSION_MOTOR_ID);
        hookMotor = new TalonFX(Constants.Climber.HOOK_RETRACT_MOTOR_ID);
     }

    }
    