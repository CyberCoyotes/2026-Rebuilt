package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

        static TalonFXConfiguration leader() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotorOutput.NeutralMode = Constants.Intake.RollerLeaderConfig.NEUTRAL_MODE;
            config.MotorOutput.Inverted = Constants.Intake.RollerLeaderConfig.INVERTED;

            config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.RollerLeaderConfig.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Constants.Intake.RollerLeaderConfig.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            config.Slot0.kS = Constants.Intake.RollerLeaderConfig.KS;
            config.Slot0.kV = Constants.Intake.RollerLeaderConfig.KV;
            config.Slot0.kP = Constants.Intake.RollerLeaderConfig.KP;

            return config;
        }

        // While in follower mode, direction is governed by the Follower request — INVERTED not set.
        /*
        static TalonFXConfiguration follower() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotorOutput.NeutralMode = Constants.Intake.RollerFollowerConfig.NEUTRAL_MODE;

            config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.RollerFollowerConfig.SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = Constants.Intake.RollerFollowerConfig.STATOR_CURRENT_LIMIT;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            return config;
        }
        */
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

            /* MotionMagic profile — starts with the fast (default) values */
            config.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.SLIDE_MM_CRUISE_VELOCITY;
            config.MotionMagic.MotionMagicAcceleration = Constants.Intake.SLIDE_MM_ACCELERATION;
            config.MotionMagic.MotionMagicJerk = Constants.Intake.SLIDE_MM_JERK;

            return config;
        }
    }

    // == Hardware =============================================================
    private final TalonFX rollerLead; // Currently the left roller motor
    // private final TalonFX rollerFollow; // Currently the right motor
    private final TalonFX slide;

    // == Control Requests =====================================================
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0);

    // Single MotionMagic request used for both fast and slow slide movement.
    // The speed difference comes from swapping the motor's MotionMagic config.
    private final MotionMagicVoltage slideRequest = new MotionMagicVoltage(0);

    // == MotionMagic Profile Configs ==========================================
    // Pre-built configs for swapping between fast and slow slide profiles.
    // Only the MotionMagic section is applied — PID, current limits, etc. stay unchanged.
    private final MotionMagicConfigs fastProfile = new MotionMagicConfigs();
    private final MotionMagicConfigs slowProfile = new MotionMagicConfigs();
    private boolean slowProfileActive = false;

    // == Status Signals =============================================================
    // Current, voltage, and temp are captured by CTRE Hoot for diagnostics.
    private final StatusSignal<Angle> slidePosition;
    private final StatusSignal<AngularVelocity> slideVelocity;

    public IntakeIOHardware() {
        rollerLead = new TalonFX(Constants.Intake.ROLLER_LEFT_MOTOR_ID, Constants.RIO_CANBUS);
        // rollerFollow = new TalonFX(Constants.Intake.ROLLER_RIGHT_MOTOR_ID, Constants.RIO_CANBUS);

        slide = new TalonFX(Constants.Intake.SLIDE_MOTOR_ID, Constants.RIO_CANBUS);

        PhoenixUtil.applyConfig("Roller Lead",   () -> rollerLead.getConfigurator().apply(RollerConfig.leader()));
        // PhoenixUtil.applyConfig("Roller Follow", () -> rollerFollow.getConfigurator().apply(RollerConfig.follower()));
        PhoenixUtil.applyConfig("Slide",         () -> slide.getConfigurator().apply(SlideConfig.slide()));

        // Build fast/slow MotionMagic configs for runtime profile swapping
        fastProfile.MotionMagicCruiseVelocity = Constants.Intake.SLIDE_MM_CRUISE_VELOCITY;
        fastProfile.MotionMagicAcceleration = Constants.Intake.SLIDE_MM_ACCELERATION;
        fastProfile.MotionMagicJerk = Constants.Intake.SLIDE_MM_JERK;

        slowProfile.MotionMagicCruiseVelocity = Constants.Intake.SLIDE_SLOW_MM_CRUISE_VELOCITY;
        slowProfile.MotionMagicAcceleration = Constants.Intake.SLIDE_SLOW_MM_ACCELERATION;
        slowProfile.MotionMagicJerk = Constants.Intake.SLIDE_SLOW_MM_JERK;

        // Cache signal references — slide needs position and velocity for MotionMagic
        // and at-target checks. Roller has no control-critical signals to read.
        slidePosition = slide.getPosition();
        slideVelocity = slide.getVelocity();

        // rollerFollow.setControl(
        //         new Follower(rollerLead.getDeviceID(), Constants.Intake.RollerFollowerConfig.FOLLOWER_ALIGNMENT));

        // Zero slide encoder at startup
        slide.setPosition(0);
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
    public void setRollerVelocity(double rps) {
        rollerLead.setControl(rollerRequest.withVelocity(rps));
    }

    @Override
    public void stopRoller() {
        rollerLead.stopMotor();
    }

    // ==== Slide Methods ====
    @Override
    public void setSlidePosition(double position) {
        if (slowProfileActive) {
            slide.getConfigurator().apply(fastProfile);
            slowProfileActive = false;
        }
        slide.setControl(slideRequest.withPosition(position));
    }

    @Override
    public void setSlidePositionSlow(double position) {
        if (!slowProfileActive) {
            slide.getConfigurator().apply(slowProfile);
            slowProfileActive = true;
        }
        slide.setControl(slideRequest.withPosition(position));
    }

    @Override
    public void stopSlide() {
        slide.stopMotor();
    }
}
