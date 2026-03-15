package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil; // ← ADDED

/**
 * IntakeIOHardware - Real hardware implementation using CTRE TalonFX motors.
 *
 * This class interfaces with:
 * - 1x TalonFX (roller) for spinning intake wheels
 * - 1x TalonFX (slide) for extending/retracting intake
 *
 * All apply() calls use PhoenixUtil.applyConfig() for retry logic — a single
 * apply() on RIO CAN bus can return OK prematurely if the device is still booting.
 *
 * @author @Isaak3
 */
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
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            config.CurrentLimits.SupplyCurrentLimit = 40.0;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40.0;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 44.25;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.2;

            config.Slot0.kP = 2.0;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kS = 0.7;

            config.MotionMagic.MotionMagicCruiseVelocity = 363;
            config.MotionMagic.MotionMagicAcceleration = 363;
            config.MotionMagic.MotionMagicJerk = 0;

            return config;
        }
    }

    // ── Hardware ───────────────────────────────────────────────────────────────
    private final TalonFX roller;
    private final TalonFX slide;

    // ── Control Requests ───────────────────────────────────────────────────────
    private final VoltageOut rollerRequest = new VoltageOut(0);

    private final MotionMagicVoltage slideRequest = new MotionMagicVoltage(0);

    // TODO: Tune velocity/accel for a slower retract profile
    private final DynamicMotionMagicVoltage slideRequestSlow = new DynamicMotionMagicVoltage(0, 4, 4);

    // ── Status Signals — 50Hz (control-critical) ──────────────────────────────
    private final StatusSignal<Angle> slidePosition;
    private final StatusSignal<AngularVelocity> slideVelocity;

    // ── Constructor ────────────────────────────────────────────────────────────
    public IntakeIOHardware() {
        roller = new TalonFX(Constants.Intake.INTAKE_ROLLER_MOTOR_ID, Constants.RIO_CANBUS);
        slide  = new TalonFX(Constants.Intake.INTAKE_SLIDE_MOTOR_ID,  Constants.RIO_CANBUS);

        // Apply configs with retry logic — bare apply() can fail silently on RIO CAN
        // bus if the device is still booting. PhoenixUtil retries up to 5 times.
        PhoenixUtil.applyConfig("Intake roller", // ← CHANGED
            () -> roller.getConfigurator().apply(RollerConfig.roller()));
        PhoenixUtil.applyConfig("Intake slide",  // ← CHANGED
            () -> slide.getConfigurator().apply(SlideConfig.slide()));

        slidePosition = slide.getPosition();
        slideVelocity = slide.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            slidePosition,
            slideVelocity
        );

        roller.optimizeBusUtilization();
        slide.optimizeBusUtilization();

        // Zero slide encoder at startup — assumes slide is fully retracted
        slide.setPosition(0.0);
    }

    // ── IO Implementation ──────────────────────────────────────────────────────
    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(slidePosition, slideVelocity);

        inputs.slidePositionRotations = slidePosition.getValueAsDouble();
        inputs.slideVelocityRPS = slideVelocity.getValueAsDouble();
    }

    // ── Roller Methods ─────────────────────────────────────────────────────────
    @Override
    public void setRollerVoltage(double volts) {
        roller.setControl(rollerRequest.withOutput(volts));
    }

    @Override
    public void stopRoller() {
        roller.stopMotor();
    }

    // ── Slide Methods ──────────────────────────────────────────────────────────
    @Override
    public void setSlidePosition(double position) {
        slide.setControl(slideRequest.withPosition(position));
    }

    @Override
    public void setSlidePositionSlow(double position) {
        slide.setControl(slideRequestSlow.withPosition(position));
    }

    @Override
    public double getSlidePosition() {
        return slidePosition.getValueAsDouble();
    }

    @Override
    public void stopSlide() {
        slide.stopMotor();
    }
}