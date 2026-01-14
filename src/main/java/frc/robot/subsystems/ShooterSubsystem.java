package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterSubsystem extends SubsystemBase{

private static final int motorOne_ID = 67;
private static final int motorTwo_ID = 69;

//speed presets
private static final int fullRPM = 6000;
private static final int halfRPM = 4000;
private static final int quarterRPM = 2000;

//Motors
private final TalonFX motorOne;
private final TalonFX motorTwo;

//Constructor(whatever that means)
public ShooterSubsystem(){
    motorOne = new TalonFX(motorOne_ID);
    motorTwo = new TalonFX(motorTwo_ID);
}

public void fullSpeed() {
motorOne.setVelocity(fullRPM);
motorTwo.setVelocity(fullRPM);
}
public void halfSpeed() {
motorOne.setVelocity(halfRPM);
motorTwo.setVelocity(halfRPM);
}
public void quarterSpeed() {
    motorOne.setVelocity(quarterRPM);
    motorTwo.setVelocity(quarterRPM);
}
public void stopShooter() {
motorOne.setVelocity(0);
motorTwo.setVelocity(0);
}

@Override
public void periodic(){

}
}