package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private final TalonFX rotatorMotor;

    //uses Kraken x44 with TalonFX interface
    IntakeSubsystem(){
        rotatorMotor = new TalonFX(66); //TODO: fix this filler PID
    }

    public void setVolts(double volts){
        rotatorMotor.setControl(new VoltageOut(volts));
    }

    public void stop(){
        rotatorMotor.setControl(new VoltageOut(0));
    }

    public StatusSignal<Voltage> getVolts(){
        return rotatorMotor.getMotorVoltage();
    }


}
