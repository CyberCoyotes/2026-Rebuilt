package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

/*
 * AUTHOR: @Isaak3
 * The shooter needs to be able to shoot the fuel and adjust the hood
 * it will use 5 motors: 
 * -- 3 motors for the main flywheels(Falcon500) TalonFX controller. ControlMode should be VelocityTorqueCurrentFOC
 * -- 1 motor for the hood movements (Minion) controlled by Talon FXS,,
 * -- 1 motor for the counter-wheel (Kraken x44);
*/


/*
* Suggested Shooter State Machine:
* enum ShooterState {
IDLE, // Flywheel off
SPINUP, // Flywheel accelerating to LOW_VEL (warmup speed)
READY, // Flywheel at target velocity, ready to shoot
EJECT, // Flywheel reverse (if needed for clearing jams)
PASSING // Flywheel at pass velocity
}

// Shooter signals readiness
if (shooter.getState() == READY && shooter.isAtVelocity()) {
indexer.setState(FEEDING); // Indexer starts feeding
shooter.setState(SHOOTING); // Shooter tracks that shot is happening
}

For logging/dashboard:
You could have:
SmartDashboard.putBoolean("Robot Shooting", isShooting());

// In ShooterSubsystem:
private double targetHoodAngle = HOME_ANGLE;

public boolean isReadyToShoot() {
return isFlywheelAtVelocity() && isHoodAtTarget();
}

public boolean isHoodAtTarget() {
return Math.abs(hoodEncoder.getPosition() - targetHoodAngle) < TOLERANCE;
}

State transitions consider both:

// Transition to READY only when BOTH are ready
if (currentState == SPINUP && isReadyToShoot()) {
setState(READY);
}

// In a Superstructure or Robot class:
public boolean isShooting() {
return shooter.getState() == READY &&
shooter.isAtVelocity() &&
indexer.getState() == FEEDING;
}


*/

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

//speed presets
private static final int fullRPS = 100;
private static final int halfRPS = 67;
private static final int quarterRPS = 33;

//Motors
private final TalonFX motorOne;
private final TalonFX motorTwo;
private final TalonFX motorThree;
private VelocityVoltage velocityVoltage;

//Constructor
public ShooterSubsystem(){
    motorOne = new TalonFX(71);
    motorTwo = new TalonFX(72);
    motorThree = new TalonFX(73); //third motor ID
}

VelocityVoltage velocityRequest = new VelocityVoltage(0);

public void fullSpeed() {
motorOne.setControl(velocityRequest.withVelocity(fullRPS));
motorTwo.setControl(velocityRequest.withVelocity(fullRPS));
}
public void halfSpeed() {
motorOne.setControl(velocityRequest.withVelocity(halfRPS));
motorTwo.setControl(velocityRequest.withVelocity(halfRPS));
}
public void quarterSpeed() {
    motorOne.setControl(velocityRequest.withVelocity(quarterRPS));
    motorTwo.setControl(velocityRequest.withVelocity(quarterRPS));
}
public void stopShooter() {
motorOne.setControl(velocityRequest.withVelocity(0));
motorTwo.setControl(velocityRequest.withVelocity(0));
}

@Override
public void periodic(){

}
}