package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants;
import frc.robot.utilities.PhoenixUtil;

/**
 * IntakeIOHardware - Real hardware implementation using CTRE TalonFX motors.
 *
 * This class interfaces with:
 * - 1x TalonFX (roller) for spinning intake wheels
 * - 1x TalonFX (slide) for extending/retracting intake
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Voltage control for roller, MotionMagic for slide
 * - All telemetry logged via AdvantageKit
 *
 * @author @Isaak3
 */


public class IntakeIOHardware implements IntakeIO {

    // == Roller Configuration =====================================
    private static class RollerConfig {

        static TalonFXConfiguration roller() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotorOutput.NeutralMode = Constants.Intake.RollerConfig.NEUTRAL_MODE;
            config.MotorOutput.Inverted = Constants.Intake.RollerConfig.INVERTED;

            config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.RollerConfig.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Constants.Intake.RollerConfig.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            return config;
        }
    }

    // == Slide Configuration ======================================
    private static class SlideConfig {

        static TalonFXConfiguration slide() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotorOutput.NeutralMode = Constants.Intake.SlideConfig.NEUTRAL_MODE;
            config.MotorOutput.Inverted = Constants.Intake.SlideConfig.INVERTED;

            config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.SlideConfig.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Constants.Intake.SlideConfig.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.SLIDE_MAX_POS;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.SlideConfig.REVERSE_SOFT_LIMIT;

            config.Slot0.kP = Constants.Intake.SlideConfig.KP;
            config.Slot0.kI = Constants.Intake.SlideConfig.KI;
            config.Slot0.kD = Constants.Intake.SlideConfig.KD;
            config.Slot0.kS = Constants.Intake.SlideConfig.KS;
            config.Slot0.kV = Constants.Intake.SlideConfig.KV;
            config.Slot0.kA = Constants.Intake.SlideConfig.KA;

            /* MotionMagic profile */
            config.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.SLIDE_MM_CRUISE_VELOCITY;
            config.MotionMagic.MotionMagicAcceleration = Constants.Intake.SLIDE_MM_ACCELERATION;
            config.MotionMagic.MotionMagicJerk = Constants.Intake.SLIDE_MM_JERK;

            return config;
        }
    }

    // == Hardware =============================================================
    private final TalonFX rollerLead; // Currently the left roller motor, but can be swapped if needed by changing motor IDs in Constants
    private final TalonFX rollerFollow;  // Currently the right roller motor
    private final TalonFX slide;

    // == Control Requests =====================================================
    private final VoltageOut rollerRequest = new VoltageOut(0);

    // MotionMagic control for slide — set position, motor holds after command ends
    private final MotionMagicVoltage slideRequest = new MotionMagicVoltage(0);

    // DynamicMotionMagic for slower slide movement 
    private final DynamicMotionMagicVoltage slideRequestSlow =
            new DynamicMotionMagicVoltage(
                    0,
                    Constants.Intake.SLIDE_SLOW_MM_CRUISE_VELOCITY,
                    Constants.Intake.SLIDE_SLOW_MM_ACCELERATION);

    // == Status Signals =============================================================
    // Current, voltage, and temp are captured by CTRE Hoot for diagnostics.
    private final StatusSignal<Angle> slidePosition;
    private final StatusSignal<AngularVelocity> slideVelocity;

    public IntakeIOHardware() {
        rollerLead = new TalonFX(Constants.Intake.ROLLER_LEFT_MOTOR_ID, Constants.RIO_CANBUS);
        rollerFollow = new TalonFX(Constants.Intake.ROLLER_RIGHT_MOTOR_ID, Constants.RIO_CANBUS);

        slide  = new TalonFX(Constants.Intake.SLIDE_MOTOR_ID, Constants.RIO_CANBUS);

        PhoenixUtil.applyConfig("Roller Lead",   () -> rollerLead.getConfigurator().apply(RollerConfig.roller()));
        PhoenixUtil.applyConfig("Roller Follow", () -> rollerFollow.getConfigurator().apply(RollerConfig.roller()));
        PhoenixUtil.applyConfig("Slide",         () -> slide.getConfigurator().apply(SlideConfig.slide()));

        // Cache signal references — slide needs position and velocity for MotionMagic
        // and at-target checks. Roller has no control-critical signals to read.
        slidePosition = slide.getPosition();
        slideVelocity = slide.getVelocity();

        rollerFollow.setControl(
                new Follower(rollerLead.getDeviceID(), Constants.Intake.RollerConfig.FOLLOWER_ALIGNMENT));

        // Zero slide encoder at startup
        // slide.setPosition(Constants.Intake.ENCODER_ZERO_POSITION);
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

    }

    // ==== Roller Methods ====
    @Override
    public void setRollerVoltage(double volts) {
        rollerLead.setControl(rollerRequest.withOutput(volts));
        // Follower will automatically oppose lead motor, so no need to set voltage here.
        // Calling the motor directly a "follower break"
        // rollerFollow.setControl(rollerRequest.withOutput(-volts)); 
    }

    @Override
    public void stopRoller() {
        rollerLead.stopMotor();
        // rollerFollow.stopMotor();
    }

    // ==== Slide Methods ====
    @Override
    public void setSlidePosition(double position) {
        slide.setControl(slideRequest.withPosition(position));
    }

    @Override
    public void setSlidePositionSlow(double position) {
        // This request carries its own cruise/accel limits, so we can tune slow
        // retract independently from the normal Motion Magic profile in config.
        slide.setControl(slideRequestSlow.withPosition(position));
    }

    // This was not following the IO pattern and was being called directly by the subsystem
    // @Override
    // public double getSlidePosition() {
    //     return slidePosition.getValueAsDouble();
    // }

    @Override
    public void stopSlide() {
        slide.stopMotor();
    }
}
