package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
// Add import for the control mode of CTRE Phoenix
import com.ctre.phoenix6.controls.PositionVoltage;

/*
 * AUTHOR: @imadinosaur1
 *
 * Purpose: Subsystem for the Climber mechanism
 * Functionality: Extend/retract climber arms
 *
 * @see Constants.Climber for hardware configuration
 */

// @SuppressWarnings("unused")

public class ClimberSubsystem extends SubsystemBase {

   // Position Constants
   private static final double MAX = 1.0; // TODO: Determine actual max position
   private static final double HOME = 0.0; // Home or retracted position
   private static final double LEVEL_ONE = 0.5; // TODO: Determine level one position
   private static final double LEVEL_TWO = 0.75; // TODO: Determine level one position

   private final PositionVoltage positionControl = new PositionVoltage(0.0).withSlot(0);

   // Hardware; commented out until motors are assigned
   private final TalonFX climbMotor;

   // Constructor
   public ClimberSubsystem() {
      climbMotor = new TalonFX(Constants.Climber.CLIMB_MOTOR_ID);
      climbMotor.setPosition(0.0);

      var config = new TalonFXConfiguration();
      // Configure motor settings as needed
      config.Slot0.kP = 24.1; // TODO: Tune the PID value
      config.Slot0.kI = 0.0;
      config.Slot0.kD = 0.0;

      // Optional: Set soft limits to prevent over-extension
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HOME;

      climbMotor.getConfigurator().apply(config);

   }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run

      // TODO: Add telemetry here
      // TODO: Add logging and tele
   }

   // Climber methods
   public void setClimberVolts(double voltage) { // Set arm to extend give it a variable and declare type
      climbMotor.setVoltage(voltage); // Voltage will range -12 to 12 to extend
   }

   public void setClimberPosition(double position) {
      climbMotor.setControl(positionControl.withPosition(position)); // Set position control to desired position
   }

   /**
    * Check if climber is at the target position
    * 
    * @param targetPosition Position to check against (in rotations)
    * @return true if climber is within tolerance of target
    */
   private boolean atPosition(double targetPosition) {
      double currentPosition = climbMotor.getPosition().getValueAsDouble();
      double tolerance = 0.5; // TODO rotations - adjust based on testing
      return Math.abs(currentPosition - targetPosition) < tolerance;
   }

   // ===== Command factory methods =====

   /*
    * Commands for manual climber movement
    * These commands run the climber motors at fixed voltages to extend or retract
    * the arms.
    * They can be used for teleoperated control or fine-tuning during testing.
    * Each command runs until interrupted, allowing for precise control over
    * climber movement.
    * They will need to be stopped manually by another command or condition.
    * 
    * @return Command to extend or retract the climber arms
    */
   public Command extendArm() {
      return run(() -> setClimberVolts(3));
   }

   public Command retractArm() {
      // Make the same as extendArm but negative voltage
      return run(() -> setClimberVolts(-3));
   }

   /*
    * Commmand for setting climber position
    * These commands moves the climber arms to a specified position using position
    * control.
    */
   public Command setHome() {
      /*
       * Use runOnce() if you want it to run only once which is ideal for position
       * commands
       * However, runOnce() can be interrupted prematurely.
       * Use run() is used here to continuously set the position until the target is
       * reached
       */
      return run(() -> setClimberPosition(HOME))
            .until(() -> atPosition(HOME));
   }

   // Move climber to level one position at set velocity (TBD)
   public Command setLevelOne() {
      return run(() -> setClimberPosition(LEVEL_ONE))
            .until(() -> atPosition(LEVEL_ONE));

   }

   // Move climber to level two position at set velocity (TBD)
   public Command setLevelTwo() {
      return run(() -> setClimberPosition(LEVEL_TWO))
            .until(() -> atPosition(LEVEL_TWO));

   }

   public Command stopClimber() {
      return runOnce(() -> setClimberVolts(0));
   }

}// End of ClimberSubsystem class
