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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.util.TalonFXConfigs;

/**
 * IntakeIOHardware - Real hardware implementation using CTRE TalonFX motors.
 *
 * This class interfaces with:
 * - 1x TalonFX (roller) for spinning intake wheels
 * - 1x TalonFX (slide) for extending/retracting intake
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Voltage control for roller
 * - Voltage control for slide (MotionMagic commented out pending tuning)
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

    // ===== Control Requests =====
    private final VoltageOut m_rollerRequest = new VoltageOut(0);
    private final VoltageOut m_slideVoltageRequest = new VoltageOut(0); // Voltage control — active
    private final MotionMagicVoltage m_slideRequest = new MotionMagicVoltage(0); // Reserved for when MotionMagic is re-enabled

    // ===== Status Signals =====
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
    private static final double SLIDE_GEAR_RATIO = 1.0; // TODO: Measure actual slide ratio

    public IntakeIOHardware() {
        m_roller = new TalonFX(Constants.Intake.INTAKE_ROLLER_MOTOR_ID, Constants.RIO_CANBUS);
        m_slide = new TalonFX(Constants.Intake.INTAKE_SLIDE_MOTOR_ID, Constants.RIO_CANBUS);

        m_roller.getConfigurator().apply(TalonFXConfigs.rollerConfig());
        m_slide.getConfigurator().apply(TalonFXConfigs.slideConfig());

        rollerVelocity = m_roller.getVelocity();
        rollerVoltage = m_roller.getMotorVoltage();
        rollerCurrent = m_roller.getSupplyCurrent();
        rollerTemp = m_roller.getDeviceTemp();

        slidePosition = m_slide.getPosition();
        slideVelocity = m_slide.getVelocity();
        slideVoltage = m_slide.getMotorVoltage();
        slideCurrent = m_slide.getSupplyCurrent();
        slideTemp = m_slide.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            rollerVelocity, rollerVoltage,
            slidePosition, slideVelocity,
            slideVoltage,
            rollerCurrent, slideCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            4.0,
            rollerTemp, slideTemp
        );

        m_roller.optimizeBusUtilization();
        m_slide.optimizeBusUtilization();

        // Zero slide encoder at startup (assumes slide is retracted at startup)
        m_slide.setPosition(0.00);
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        // refreshAll commented out — all input reads are disabled.
        // Re-enable alongside the reads below when telemetry is needed.
        // BaseStatusSignal.refreshAll(
        //     rollerVelocity, rollerVoltage, rollerCurrent, rollerTemp,
        //     slidePosition, slideVelocity, slideVoltage, slideCurrent, slideTemp
        // );

        // inputs.rollerVelocityRPS = rollerVelocity.getValueAsDouble();
        // inputs.rollerAppliedVolts = rollerVoltage.getValueAsDouble();
        // inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
        // inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();

        // inputs.slidePositionRotations = slidePosition.getValueAsDouble();
        // inputs.slideVelocityRPS = slideVelocity.getValueAsDouble();
        // inputs.slideAppliedVolts = slideVoltage.getValueAsDouble();
        // inputs.slideCurrentAmps = slideCurrent.getValueAsDouble();
        // inputs.slideTempCelsius = slideTemp.getValueAsDouble();

        // inputs.intakeDistance = getIntakeDistance();
        // inputs.intakeTarget = intakeTargetClose();
    }

    // ===== Roller methods =====

    @Override
    public void setRollerSpeed(double speed) {
        m_roller.setControl(m_rollerRequest.withOutput(speed));
    }

    @Override
    public double getRollerVolts() {
        return m_roller.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void stopRoller() {
        m_roller.setControl(new VoltageOut(0));
    }

    // ===== Slide methods =====

    /**
     * Position control via MotionMagic — currently disabled pending tuning.
     * Use setSlideVoltage() instead.
     */
    @Override
    public void setSlidePosition(double position) {
        // m_slide.setControl(m_slideRequest.withPosition(position)); // Re-enable once MotionMagic is tuned
    }

    /**
     * Voltage control for slide motor.
     * Positive volts extends, negative volts retracts.
     * Always follow up with setSlideVoltage(0) when done.
     *
     * @param volts Output voltage (-12.0 to 12.0)
     */
    @Override
    public void setSlideVoltage(double volts) {
        m_slide.setControl(m_slideVoltageRequest.withOutput(volts));
    }

    @Override
    public double getSlidePosition() {
        return m_slide.getPosition().getValueAsDouble();
    }

    // ===== Intake sensor methods =====
    // public double getIntakeDistance() {
    //     return s_intakeTOF.isRangeValid() ? s_intakeTOF.getRange() : Double.NaN;
    // }

    // public boolean intakeTargetClose() {
    //     return (s_intakeTOF.getRange() <= IntakeSubsystem.INTAKE_THRESHOLD) && s_intakeTOF.isRangeValid();
    // }

} // end of class