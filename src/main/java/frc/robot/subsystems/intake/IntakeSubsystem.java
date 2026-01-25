package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{

    private final TalonFX m_rotator; ///rotates the roller
    private final TalonFX m_slide; //moves the intake forward and backwards
    private final VelocityVoltage m_rotator_request;
    private final MotionMagicVoltage m_slide_request;
    private final TimeOfFlight s_intake; // detects objects near intake
    private final TimeOfFlight s_indexer; // detects when ball capacity meets standards for indexer handoff 

    private final int INTAKE_SENSOR_ID = 12345;
    private final int INTAKE_THRESHOLD = 1000; //mm, around four inches

    private final int INDEXER_SENSOR_ID = 234;
    private final int INDEXER_THRESHOLD = 67; //mm

    private final double JAM_CURRENT_THRESHOLD = 20.0; // current should be under this
    private final double JAM_VELOCITY_THRESHOLD = 0.5; // velocity should be over this

    public final double SLIDE_EXTENDED_POSITION = 0.5;
    private final double SLIDE_RESTING_POSITION = 0;
    public final double ROTATOR_RUNNING_VELOCITY = 0.5;

    public final double ROTATOR_MAX_VELOCITY = 1;
    
    //uses Kraken x44 with TalonFX interface
    IntakeSubsystem(){
        m_rotator = new TalonFX(Constants.Intake.INTAKE_ROTATOR_MOTOR_ID); // intaking "wheel"
        m_slide = new TalonFX(Constants.Intake.INTAKE_SLIDE_A_MOTOR_ID);
        m_rotator_request = new VelocityVoltage(0).withSlot(0);
        m_slide_request = new MotionMagicVoltage(0);
        s_intake = new TimeOfFlight(INTAKE_SENSOR_ID);
        s_indexer = new TimeOfFlight(INDEXER_SENSOR_ID);

        var rotatorConfigs = new TalonFXConfiguration();
        var slideConfigs = new TalonFXConfiguration();

        var rotatorSlot0 = rotatorConfigs.Slot0; // tuner reccomended values
        rotatorSlot0.kS = 0.1; // add 0.1 V to overcome static friction
        rotatorSlot0.kV = 0.12; // velocity target of 1 rps = 0.12 V output
        rotatorSlot0.kP = 0.11; // error of 1 rps = 0.11 V output
        rotatorSlot0.kI = 0; // no output
        rotatorSlot0.kD = 0; // no output

        var slideSlot0 = slideConfigs.Slot0;
        slideSlot0.kS = 0.1;
        slideSlot0.kV = 0.12;
        slideSlot0.kP = 0.11;
        slideSlot0.kI = 0;
        slideSlot0.kD = 0;

        var motionMagicConfigs = slideConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;

        m_rotator.getConfigurator().apply(rotatorSlot0);
        m_slide.getConfigurator().apply(slideSlot0);
    }

    //rotator methods
    public void setRotatorVelocity(double velocity){
        m_rotator.setControl(m_rotator_request.withVelocity(velocity));
    }

    public StatusSignal<Voltage> getRotatorVolts(){
        return m_rotator.getMotorVoltage();
    }

    public void stopRotator(){
        m_rotator.setControl(new VoltageOut(0));
    }

    //slide methods
    public void setSlidePosition(double position){
        m_slide.setControl(m_slide_request.withPosition(position));
    }

    public StatusSignal<Angle> getSlidePosition(){
        return m_slide.getPosition();
    }
    
    //intake sensor methods
    public double getIntakeDistance(){
        return s_intake.getRange();
    }

     public boolean intakeTargetClose(){
        return (s_intake.getRange() <= INTAKE_THRESHOLD) && s_intake.isRangeValid();
    }

    //indexer sensor methods
    public double getIndexerDistance(){
        return s_indexer.getRange();
    }

    //is something close to indexer TOF
    public boolean indexerTargetClose(){
        return (s_indexer.getRange() <= INDEXER_THRESHOLD) && s_indexer.isRangeValid();
    }

    //multi-hardware methods
    public void toRestingState(){
        if (!isJammed()){
        setSlidePosition(SLIDE_RESTING_POSITION); // if ball is stuck, moving slide to rest is bad
        }
        setRotatorVelocity(0);
    }

    // TODO tune
    public boolean isJammed(){
        double current = m_rotator.getSupplyCurrent().getValueAsDouble();
        double velocity = m_rotator.getVelocity().getValueAsDouble();

        return (current >= JAM_CURRENT_THRESHOLD) && (velocity <= JAM_VELOCITY_THRESHOLD);
  }


}
