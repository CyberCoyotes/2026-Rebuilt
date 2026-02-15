package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.util.TalonFXConfigs;

/**
 * IntakeIOTalonFX - Real hardware implementation using CTRE TalonFX motors.
 *
 * This class interfaces with:
 * - 1x TalonFX (roller) for spinning intake wheels
 * - 1x TalonFX (slide) for extending/retracting intake
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Percent output control for roller (simple and responsive)
 * - Position control for slide (precise positioning)
 * - Optimized status signal updates for performance
 * - All telemetry logged via AdvantageKit
 *
 * @author @Isaak3
 */

@SuppressWarnings("unused") // Remove when we approach comp ready code

public class IntakeIOHardware implements IntakeIO {

    // ===== Hardware =====
    private final TalonFX m_roller;
    private final TalonFX m_slide;
    private final TimeOfFlight s_intake; // detects objects near intake
    private final TimeOfFlight s_indexer; // detects when ball capacity meets standards for indexer handoff 
    private final CANBus kCANBus = new CANBus("rio");


    // ===== Control Requests =====
    private final VoltageOut m_roller_request = new VoltageOut(0);
    private final MotionMagicVoltage m_slide_request = new MotionMagicVoltage(0); //TODO add limit switches
   // ===== Status Signals (for efficient reading) =====
    // Roller
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerCurrent;
    private final StatusSignal<Temperature> rollerTemp;

    // Slide
    private final StatusSignal<Angle> slidePosition;
    private final StatusSignal<AngularVelocity> slideVelocity;
    private final StatusSignal<Voltage> slideVoltage;
    private final StatusSignal<Current> slideCurrent;
    private final StatusSignal<Temperature> slideTemp;

    // ===== Conversion Constants =====
    /** Gear ratio from slide motor to slide mechanism (motor rotations per slide extension) */
    private static final double SLIDE_GEAR_RATIO = 10.0;  // TODO: Measure actual ratio

    /**
     * Creates a new IntakeIOTalonFX instance.
     * Configures motors to known good states.
     */
    public IntakeIOHardware() {
        // Create motor objects
        m_roller = new TalonFX(Constants.Intake.INTAKE_ROLLER_MOTOR_ID, kCANBus); // TODO add new CANbus arg 
        m_slide = new TalonFX(Constants.Intake.INTAKE_SLIDE_MOTOR_ID, kCANBus); // TODO add new CANbus arg 

        //create sensor objects
        s_intake = new TimeOfFlight(IntakeConstants.INTAKE_SENSOR_ID);
        s_indexer = new TimeOfFlight(IntakeConstants.INDEXER_SENSOR_ID);

        // Apply configurations from centralized config class
        m_roller.getConfigurator().apply(TalonFXConfigs.intakeConfig());
        m_slide.getConfigurator().apply(TalonFXConfigs.intakeConfig()); //TODO should both motors have the same config?

        // Get status signals for efficient reading
        // Roller
        rollerVelocity = m_roller.getVelocity();
        rollerVoltage = m_roller.getMotorVoltage();
        rollerCurrent = m_roller.getSupplyCurrent();
        rollerTemp = m_roller.getDeviceTemp();

        // Slide
        slidePosition = m_slide.getPosition();
        slideVelocity = m_slide.getVelocity();
        slideVoltage = m_slide.getMotorVoltage();
        slideCurrent = m_slide.getSupplyCurrent();
        slideTemp = m_slide.getDeviceTemp();

        // Configure update frequencies for better performance
        // Critical signals: 100Hz, Less critical: 50Hz, Temperature: 4Hz
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,  // 100Hz for velocity/position/voltage (critical for control)
            rollerVelocity, rollerVoltage,
            slidePosition, slideVelocity, slideVoltage
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,  // 50Hz for current
            rollerCurrent, slideCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            4.0,  // 4Hz for temperature
            rollerTemp, slideTemp
        );

        // Optimize bus utilization
        m_roller.optimizeBusUtilization();
        m_slide.optimizeBusUtilization();

        // Zero slide encoder at startup (assumes slide is retracted at startup)
        m_slide.setPosition(0.0);
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        // Refresh all status signals efficiently
        BaseStatusSignal.refreshAll(
            rollerVelocity, rollerVoltage, rollerCurrent, rollerTemp,
            slidePosition, slideVelocity, slideVoltage, slideCurrent, slideTemp
        );

        // Roller motor data
        inputs.rollerVelocityRPS = rollerVelocity.getValueAsDouble();
        inputs.rollerAppliedVolts = rollerVoltage.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();

        // Slide motor data
        inputs.slidePositionRotations = slidePosition.getValueAsDouble();
        inputs.slideVelocityRPS = slideVelocity.getValueAsDouble();
        inputs.slideAppliedVolts = slideVoltage.getValueAsDouble();
        inputs.slideCurrentAmps = slideCurrent.getValueAsDouble();
        inputs.slideTempCelsius = slideTemp.getValueAsDouble();

        //sensor data
        inputs.intakeDistance = getIntakeDistance();
        inputs.intakeTarget = intakeTargetClose();
        inputs.indexerDistance = getIndexerDistance();
        inputs.indexerTarget = indexerTargetClose();
    }


    
    //roller methods
    public void setRollerSpeed(double speed){
        m_roller.setControl(m_roller_request.withOutput(speed));
    }

    public double getRollerVolts(){
        return m_roller.getMotorVoltage().getValueAsDouble();
    }

    public void stopRoller(){
        m_roller.setControl(new VoltageOut(0));
    }

    //slide methods
    public void setSlidePosition(double position){
        m_slide.setControl(m_slide_request.withPosition(position));
    }

    public double getSlidePosition(){
        return m_slide.getPosition().getValueAsDouble();
    }
    
    //intake sensor methods
    public double getIntakeDistance(){
        return s_intake.isRangeValid() ? s_intake.getRange(): Double.NaN; //only gets the range if the range is valid, if not 
    }

     public boolean intakeTargetClose(){
        return (s_intake.getRange() <= IntakeConstants.INTAKE_THRESHOLD) && s_intake.isRangeValid();
    }

    //indexer sensor methods
    public double getIndexerDistance(){
        return s_indexer.isRangeValid() ? s_indexer.getRange(): Double.NaN; //only gets the range if the range is valid, if not 
    }

    //returns true if is something close to indexer TOF
    // /* */
    public boolean indexerTargetClose(){
        return (s_indexer.getRange() <= IntakeConstants.INDEXER_THRESHOLD) && s_indexer.isRangeValid();
    }
    

    //multi-hardware methods
    public void toRestingState(){
        if (!isJammed()){
        setSlidePosition(IntakeConstants.SLIDE_RESTING_POSITION); // if ball is stuck, moving slide to rest is bad
        }
        setRollerSpeed(0);
    }

    // TODO tune
    public boolean isJammed(){
        double current = m_roller.getSupplyCurrent().getValueAsDouble();
        double velocity = m_roller.getVelocity().getValueAsDouble();

        return (current >= IntakeConstants.JAM_CURRENT_THRESHOLD) && (velocity <= IntakeConstants.JAM_VELOCITY_THRESHOLD);
  }
}
