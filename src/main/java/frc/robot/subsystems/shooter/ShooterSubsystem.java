package frc.robot.subsystems.shooter;

/*
 * AUTHOR: @Isaak3
 * The shooter needs to be able to shoot the fuel and adjust the hood
 * it will use 5 motors: 3 for the main flywheels(Falcon500), 1 for the hood movements(Minion),
 * and 1 for the counter-wheel(Kraken x44);
 */

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@SuppressWarnings("unused")

public class ShooterSubsystem extends SubsystemBase{

//speed presets
private static final double fullVel = 106.3;
private static final double octVel = 85.1;
private static final int eject = -50;

//Motors
private final TalonFX flywheelA;
private final TalonFX flywheelB;
private final TalonFX flywheelC;
private final TalonFX counterWheel;
private final TalonFX hoodMotor;

//State
private shooterState currentState = shooterState.IDLE;

public enum shooterState {
IDLE,
PREPARING,
SHOOTING,
EJECTING
}

//Constructor
public ShooterSubsystem(){
    flywheelA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID);
    flywheelB = new TalonFX(Constants.Shooter.FLYWHEEL_B_MOTOR_ID);
    flywheelC = new TalonFX(Constants.Shooter.FLYWHEEL_C_MOTOR_ID);
    counterWheel = new TalonFX(Constants.Shooter.COUNTER_WHEEL_MOTOR_ID);
    hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);
}

//Control Modes
PositionVoltage positionVoltage = new PositionVoltage(0);
VelocityTorqueCurrentFOC torqueRequest = new VelocityTorqueCurrentFOC(0);

//Methods
public void fullSpeed() {
    flywheelA.setControl(torqueRequest.withVelocity(fullVel));
    flywheelB.setControl(torqueRequest.withVelocity(fullVel));
    flywheelC.setControl(torqueRequest.withVelocity(fullVel));
}
public void eighthSpeed() {
    flywheelA.setControl(torqueRequest.withVelocity(octVel));
    flywheelB.setControl(torqueRequest.withVelocity(octVel));
    flywheelC.setControl(torqueRequest.withVelocity(octVel));
}

public void setHoodStart() {
   hoodMotor.setControl(positionVoltage.withPosition(0));
}

public void stopShooter() {
    flywheelA.setControl(torqueRequest.withVelocity(0));
    flywheelB.setControl(torqueRequest.withVelocity(0));
    flywheelC.setControl(torqueRequest.withVelocity(0));
}



@Override
public void periodic(){
    switch (currentState) {
     case IDLE : 
        flywheelA.setControl(torqueRequest.withVelocity(0));
        flywheelB.setControl(torqueRequest.withVelocity(0));
        flywheelC.setControl(torqueRequest.withVelocity(0));
     break;

     case PREPARING :
        flywheelA.setControl(torqueRequest.withVelocity(octVel));
        flywheelB.setControl(torqueRequest.withVelocity(octVel));
        flywheelC.setControl(torqueRequest.withVelocity(octVel));
        Commands.waitSeconds(1);
     currentState = shooterState.SHOOTING;
     break;

     case SHOOTING : 
        flywheelA.setControl(torqueRequest.withVelocity(octVel));
        flywheelB.setControl(torqueRequest.withVelocity(octVel));
        flywheelC.setControl(torqueRequest.withVelocity(octVel));
     break;
   
     case EJECTING : 
        flywheelA.setControl(torqueRequest.withVelocity(eject));
        flywheelB.setControl(torqueRequest.withVelocity(eject));
        flywheelC.setControl(torqueRequest.withVelocity(eject));
     break;
    }
}
//Command Factories
public Command shootCommand() {
    return runOnce(() -> currentState = shooterState.PREPARING)
    .withName("ShooterStart");
}

public Command ejectCommand() {
    return runOnce(() -> currentState = shooterState.EJECTING)
    .withName("ShooterEject");
}

public Command idleCommand() {
    return runOnce(() -> currentState = shooterState.IDLE)
    .withName("ShooterIdle");
}


}