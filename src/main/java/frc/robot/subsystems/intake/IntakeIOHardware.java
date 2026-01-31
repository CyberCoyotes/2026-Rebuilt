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
 * - 1x TalonFX (rotator) for spinning intake wheels
 * - 1x TalonFX (slide) for extending/retracting intake
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Percent output control for rotator (simple and responsive)
 * - Position control for slide (precise positioning)
 * - Optimized status signal updates for performance
 * - All telemetry logged via AdvantageKit
 *
 * @author @Isaak3
 */

@SuppressWarnings("unused") // Remove when we approach comp ready code

public class IntakeIOHardware implements IntakeIO {

    // ===== Hardware =====
    private final TalonFX m_rotator;
    private final TalonFX m_slide;
    private final TimeOfFlight s_intake; // detects objects near intake
    private final TimeOfFlight s_indexer; // detects when ball capacity meets standards for indexer handoff 
    private final CANBus kCANBus = new CANBus("rio");


    // ===== Control Requests =====
    private final DutyCycleOut m_rotator_request = new DutyCycleOut(0);
    private final MotionMagicVoltage m_slide_request = new MotionMagicVoltage(0); //TODO add limit switches
   // ===== Status Signals (for efficient reading) =====
    // Rotator
    private final StatusSignal<AngularVelocity> rotatorVelocity;
    private final StatusSignal<Voltage> rotatorVoltage;
    private final StatusSignal<Current> rotatorCurrent;
    private final StatusSignal<Temperature> rotatorTemp;

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
        m_rotator = new TalonFX(Constants.Intake.INTAKE_ROTATOR_MOTOR_ID, kCANBus); // TODO add new CANbus arg 
        m_slide = new TalonFX(Constants.Intake.INTAKE_SLIDE_A_MOTOR_ID, kCANBus); // TODO add new CANbus arg 

        //create sensor objects
        s_intake = new TimeOfFlight(IntakeConstants.INTAKE_SENSOR_ID);
        s_indexer = new TimeOfFlight(IntakeConstants.INDEXER_SENSOR_ID);

        // Apply configurations from centralized config class
        m_rotator.getConfigurator().apply(TalonFXConfigs.intakeConfig());
        m_slide.getConfigurator().apply(TalonFXConfigs.intakeConfig()); //TODO should both motors have the same config?

        // Get status signals for efficient reading
        // Rotator
        rotatorVelocity = m_rotator.getVelocity();
        rotatorVoltage = m_rotator.getMotorVoltage();
        rotatorCurrent = m_rotator.getSupplyCurrent();
        rotatorTemp = m_rotator.getDeviceTemp();

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
            rotatorVelocity, rotatorVoltage,
            slidePosition, slideVelocity, slideVoltage
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,  // 50Hz for current
            rotatorCurrent, slideCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            4.0,  // 4Hz for temperature
            rotatorTemp, slideTemp
        );

        // Optimize bus utilization
        m_rotator.optimizeBusUtilization();
        m_slide.optimizeBusUtilization();

        // Zero slide encoder at startup (assumes slide is retracted at startup)
        m_slide.setPosition(0.0);
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        // Refresh all status signals efficiently
        BaseStatusSignal.refreshAll(
            rotatorVelocity, rotatorVoltage, rotatorCurrent, rotatorTemp,
            slidePosition, slideVelocity, slideVoltage, slideCurrent, slideTemp
        );

        // Rotator motor data
        inputs.rotatorVelocityRPS = rotatorVelocity.getValueAsDouble();
        inputs.rotatorAppliedVolts = rotatorVoltage.getValueAsDouble();
        inputs.rotatorCurrentAmps = rotatorCurrent.getValueAsDouble();
        inputs.rotatorTempCelsius = rotatorTemp.getValueAsDouble();

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


    
    //rotator methods
    public void setRotatorSpeed(double speed){
        m_rotator.setControl(m_rotator_request.withOutput(speed));
    }

    public double getRotatorVolts(){
        return m_rotator.getMotorVoltage().getValueAsDouble();
    }

    public void stopRotator(){
        m_rotator.setControl(new VoltageOut(0));
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
    public boolean indexerTargetClose(){
        return (s_indexer.getRange() <= IntakeConstants.INDEXER_THRESHOLD) && s_indexer.isRangeValid();
    }

    //multi-hardware methods
    public void toRestingState(){
        if (!isJammed()){
        setSlidePosition(IntakeConstants.SLIDE_RESTING_POSITION); // if ball is stuck, moving slide to rest is bad
        }
        setRotatorSpeed(0);
    }

    // TODO tune
    public boolean isJammed(){
        double current = m_rotator.getSupplyCurrent().getValueAsDouble();
        double velocity = m_rotator.getVelocity().getValueAsDouble();

        return (current >= IntakeConstants.JAM_CURRENT_THRESHOLD) && (velocity <= IntakeConstants.JAM_VELOCITY_THRESHOLD);
  }
}
