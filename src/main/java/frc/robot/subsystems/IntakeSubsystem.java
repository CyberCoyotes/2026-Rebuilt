package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private final TalonFX m_rotator;
    private final VelocityVoltage m_request;

    //uses Kraken x44 with TalonFX interface
    IntakeSubsystem(){
        m_rotator = new TalonFX(66); //TODO: filler ID
        m_request = new VelocityVoltage(0).withSlot(0);

        var rotatorConfigs = new TalonFXConfiguration();

        var slot0Configs = rotatorConfigs.Slot0; // tuner reccomended values
        slot0Configs.kS = 0.1; // add 0.1 V to overcome static friction
        slot0Configs.kV = 0.12; // velocity target of 1 rps = 0.12 V output
        slot0Configs.kP = 0.11; // error of 1 rps = 0.11 V output
        slot0Configs.kI = 0; // no output
        slot0Configs.kD = 0; // no output

        m_rotator.getConfigurator().apply(slot0Configs);
    }

    public void setVolts(double velocity){
        m_rotator.setControl(m_request.withVelocity(velocity).withFeedForward(0.5));
    }

    public void stop(){
        m_rotator.setControl(new VoltageOut(0));
    }

    public StatusSignal<Voltage> getVolts(){
        return m_rotator.getMotorVoltage();
    }


}
