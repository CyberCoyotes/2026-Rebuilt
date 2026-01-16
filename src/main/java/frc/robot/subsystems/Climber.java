package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

/* 
 *  
 * AUTHOR: @imadinosaur1
 * 
 * I moved your file to the correct branch.
 * Please complete the code below before submitting a pull request to the main repository.
 * 
 */
  // 2 motors; Kraken 60x for extension, Minion for hook movement

public class Climber extends SubsystemBase {
    
    // Constants
    private static final int ARM_MOTOR_ID = 52;
    private static final int HOOK_MOTOR_ID = 53;

    // Speed Constants
    private static final double HORNS_UP = 0.5;
    private static final double HORNS_DOWN = -0.5;
    private static final double PARADE_REST = 0.0;

    // Threshold for considering the climber "ready"
    private static final double READY_VELOCITY_THRESHOLD = 3000.0; // RPM

    // Hardware; commented out until motors are assigned
     private final TalonFX armMotor;
     private final TalonFX hookMotor;

    }
    