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

    // ==== Slide Configuration ====
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
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

            /* Tune PID values for position control of the Slide motor */
            config.Slot0.kP = 2.0;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;

            /* MotionMagic profile - TODO: Tune the MotionMagic parameters for smooth and responsive slide movement. */
            config.MotionMagic.MotionMagicCruiseVelocity = 363; // 960;
            config.MotionMagic.MotionMagicAcceleration = 363;
            config.MotionMagic.MotionMagicJerk = 0;

            return config;
        }
    }

    // ==== Hardware ====
    private final TalonFX roller;
    private final TalonFX slide;

    // ==== Control Requests ====
    private final VoltageOut rollerRequest = new VoltageOut(0);

    // MotionMagic control for slide — set position, motor holds after command ends
    private final MotionMagicVoltage slideRequest = new MotionMagicVoltage(0);

    // DynamicMotionMagic for slower slide movement 
    // TODO: Tune these for a slower retract profile
    private final DynamicMotionMagicVoltage slideRequestSlow = new DynamicMotionMagicVoltage(0, 4, 4);
                                                              // (position=0, velocity=16, accel=16, jerk=0)

    // ==== Status Signals — 50Hz (control-critical) ====
    // Current, voltage, and temp are captured by CTRE Hoot for diagnostics.
    private final StatusSignal<Angle> slidePosition;
    private final StatusSignal<AngularVelocity> slideVelocity;

    public IntakeIOHardware() {
        roller = new TalonFX(Constants.Intake.INTAKE_ROLLER_MOTOR_ID, Constants.RIO_CANBUS);
        slide  = new TalonFX(Constants.Intake.INTAKE_SLIDE_MOTOR_ID, Constants.RIO_CANBUS);

        roller.getConfigurator().apply(RollerConfig.roller());
        slide.getConfigurator().apply(SlideConfig.slide());

        // Cache signal references — slide needs position and velocity for MotionMagic
        // and at-target checks. Roller has no control-critical signals to read.
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

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        // Refresh cached signals before reading — same pattern as IndexerIOHardware.
        // Without this, slidePositionRotations is always 0 (startup value), so
        // isSlideFullyExtended() / isSlideFullyRetracted() never update correctly.
        BaseStatusSignal.refreshAll(slidePosition, slideVelocity);

        // Slide position and velocity — needed every cycle for MotionMagic and at-target checks
        inputs.slidePositionRotations = slidePosition.getValueAsDouble();
        inputs.slideVelocityRPS = slideVelocity.getValueAsDouble();

        // Sensor data — uncomment when hardware is added
        // inputs.intakeDistance = getIntakeDistance();
        // inputs.hasGamePiece = intakeTargetClose();
    }

    // ==== Roller Methods ====
    @Override
    public void setRollerVoltage(double volts) {
        roller.setControl(rollerRequest.withOutput(volts));
    }

    @Override
    public void stopRoller() {
        roller.stopMotor();
    }

    // ==== Slide Methods ====
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