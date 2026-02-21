package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants;

public class IntakeIOHardware implements IntakeIO {

    // ── Roller Configuration ───────────────────────────────────────────────────
    private static class RollerConfig {

        static TalonFXConfiguration roller() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            config.CurrentLimits.SupplyCurrentLimit = 40.0;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40.0;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            return config;
        }
    }

    // ── Slide Configuration ────────────────────────────────────────────────────
    private static class SlideConfig {

        static TalonFXConfiguration slide() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            config.CurrentLimits.SupplyCurrentLimit = 40.0;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40.0;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 44.45;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

            config.Slot0.kP = 2.0; // TODO: Tune
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;

            // MotionMagic profile
            config.MotionMagic.MotionMagicCruiseVelocity = 16; // TODO: Tune
            config.MotionMagic.MotionMagicAcceleration = 16;   // TODO: Tune
            config.MotionMagic.MotionMagicJerk = 0;

            return config;
        }
    }

    // ── Hardware ───────────────────────────────────────────────────────────────
    private final TalonFX m_roller;
    private final TalonFX m_slide;

    // ── Control Requests ───────────────────────────────────────────────────────
    private final VoltageOut m_rollerRequest = new VoltageOut(0);
    private final MotionMagicVoltage m_slideRequest = new MotionMagicVoltage(0);

    // ── Status Signals — 50Hz (control-critical) ───────────────────────────────
    // Current, voltage, and temp are captured by CTRE Hoot for diagnostics.
    private final StatusSignal<Angle> slidePosition;
    private final StatusSignal<AngularVelocity> slideVelocity;

    public IntakeIOHardware() {
        m_roller = new TalonFX(Constants.Intake.INTAKE_ROLLER_MOTOR_ID, Constants.RIO_CANBUS);
        m_slide  = new TalonFX(Constants.Intake.INTAKE_SLIDE_MOTOR_ID, Constants.RIO_CANBUS);

        m_roller.getConfigurator().apply(RollerConfig.roller());
        m_slide.getConfigurator().apply(SlideConfig.slide());

        // Cache signal references — slide needs position and velocity for MotionMagic
        // and at-target checks. Roller has no control-critical signals to read.
        slidePosition = m_slide.getPosition();
        slideVelocity = m_slide.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            slidePosition,
            slideVelocity
        );

        m_roller.optimizeBusUtilization();
        m_slide.optimizeBusUtilization();

        // Zero slide encoder at startup — assumes slide is fully retracted
        m_slide.setPosition(0.0);
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        // Slide position and velocity — needed every cycle for MotionMagic and at-target checks
        inputs.slidePositionRotations = slidePosition.getValueAsDouble();
        inputs.slideVelocityRPS = slideVelocity.getValueAsDouble();

        // Sensor data — uncomment when hardware is added
        // inputs.intakeDistance = getIntakeDistance();
        // inputs.hasGamePiece = intakeTargetClose();
    }

    // ── Roller Methods ─────────────────────────────────────────────────────────
    @Override
    public void setRollerVoltage(double volts) {
        m_roller.setControl(m_rollerRequest.withOutput(volts));
    }

    @Override
    public void stopRoller() {
        m_roller.stopMotor();
    }

    // ── Slide Methods ──────────────────────────────────────────────────────────
    @Override
    public void setSlidePosition(double position) {
        m_slide.setControl(m_slideRequest.withPosition(position));
    }

    @Override
    public void stopSlide() {
        m_slide.stopMotor();
    }
}