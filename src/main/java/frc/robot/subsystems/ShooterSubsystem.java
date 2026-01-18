package frc.robot.subsystems;

/*
 * AUTHOR: @Isaak3
 * The shooter needs to be able to shoot the fuel and adjust the hood
 * it will use 5 motors: 3 for the main flywheels(Falcon500), 1 for the hood movements(Minion),
 * and 1 for the counter-wheel(Kraken x44);
 */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class ShooterSubsystem extends SubsystemBase{

private static final int motorOne_ID = 67;
private static final int motorTwo_ID = 69;

//speed presets
private static final int fullRPS = 100;
private static final int halfRPS = 67;
private static final int quarterRPS = 33;

//Motors
private final TalonFX motorOne;
private final TalonFX motorTwo;

//Constructor
public ShooterSubsystem(){
    motorOne = new TalonFX(motorOne_ID);
    motorTwo = new TalonFX(motorTwo_ID);
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