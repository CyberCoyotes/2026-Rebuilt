package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

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

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            config.CurrentLimits.SupplyCurrentLimit = 40.0;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40.0;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            return config;
        }
    }

    // == Slide Configuration ======================================
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
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.SLIDE_MAX_POS;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; 

            /* TODO These new re-tuning for position control of the new slide mechanism */
            config.Slot0.kP = 2.0; 
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kS = 0.7;

            /* MotionMagic profile */
            config.MotionMagic.MotionMagicCruiseVelocity = 363; // 960;
            config.MotionMagic.MotionMagicAcceleration = 363;
            config.MotionMagic.MotionMagicJerk = 0;

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
    // TODO: Revisit this with the new slides to try for new retract profile
    private final DynamicMotionMagicVoltage slideRequestSlow = new DynamicMotionMagicVoltage(0, 4, 4);
                                                              // (position=0, velocity=16, accel=16, jerk=0)

    // == Status Signals ===============================================================
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

        rollerFollow.setControl(new Follower(rollerLead.getDeviceID(), MotorAlignmentValue.Opposed));

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