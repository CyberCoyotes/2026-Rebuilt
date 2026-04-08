package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.utilities.PhoenixUtil;

/**
 * ShooterIOHardware - Real hardware implementation for the shooter subsystem.
 *
 * CAN OPTIMIZATION NOTES:
 * - Only control-critical signals are read every cycle (50Hz)
 * - Current, voltage, and other diagnostics are captured by CTRE Hoot logs
 * - Through Bore encoder health is checked at 10Hz for runtime safety
 * - optimizeBusUtilization() disables all unneeded status frames
 * - Followers are set AFTER optimizeBusUtilization() to preserve control link
 * - All apply() calls use PhoenixUtil.applyConfig() for retry logic
 *
 * @see Constants.Shooter for hardware configuration
 */
public class ShooterIOHardware implements ShooterIO {

  // == Flywheel Configuration ==========================================
  private static class FlywheelConfig {

    /*
     * While in follower mode, motor direction is governed by the Follower request.
     * Keep follower motor config simple because it is not intended to run closed-loop independently.
     */
    static TalonFXConfiguration follower() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.MotorOutput.NeutralMode = Constants.Flywheel.FollowerConfig.NEUTRAL_MODE;

      config.CurrentLimits.SupplyCurrentLimit = Constants.Flywheel.SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = Constants.Flywheel.STATOR_CURRENT_LIMIT;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      return config;
    }

    static TalonFXConfiguration leader() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimit = Constants.Flywheel.SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = Constants.Flywheel.STATOR_CURRENT_LIMIT;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.MotorOutput.NeutralMode = Constants.Flywheel.LeaderConfig.NEUTRAL_MODE;
      config.MotorOutput.Inverted = Constants.Flywheel.LeaderConfig.INVERTED;

      config.MotionMagic.MotionMagicAcceleration = Constants.Flywheel.MM_ACCELERATION_RPS_PER_SEC;
      config.MotionMagic.MotionMagicJerk = Constants.Flywheel.MM_JERK_RPS_PER_SEC_CUBED;
       
      // After doing addition research, these are not used in current motor control request
      config.MotionMagic.MotionMagicExpo_kV = Constants.Flywheel.MM_EXPO_KV;
      config.MotionMagic.MotionMagicExpo_kA = Constants.Flywheel.MM_EXPO_KA;

      // Slot 0 gains for Motion Magic velocity control.
      config.Slot0.kP = Constants.Flywheel.KP;
      config.Slot0.kV = Constants.Flywheel.KV;
      config.Slot0.kA = Constants.Flywheel.KA;
      config.Slot0.kD = Constants.Flywheel.KD;

      return config;
    }
  }

  // == Hood Configuration ==========================================================
  private static class HoodConfig {

    static TalonFXSConfiguration hood() {
      TalonFXSConfiguration config = new TalonFXSConfiguration();

      config.Commutation.MotorArrangement = Constants.Hood.HoodConfig.MOTOR_ARRANGEMENT;
      config.MotorOutput.NeutralMode = Constants.Hood.HoodConfig.NEUTRAL_MODE;
      config.MotorOutput.Inverted = Constants.Hood.HoodConfig.INVERTED;

      // Voltage limits capped for safe hood movement and plenty fast for short-range repositioning.
      config.Voltage.PeakForwardVoltage = Constants.Hood.PEAK_FORWARD_VOLTAGE;
      config.Voltage.PeakReverseVoltage = Constants.Hood.PEAK_REVERSE_VOLTAGE;

      config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      // Position PID - Slot 0
      config.Slot0.kP = Constants.Hood.KP;
      config.Slot0.kI = Constants.Hood.KI;
      config.Slot0.kD = Constants.Hood.KD;
      config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.CRUISE_VELOCITY;
      config.MotionMagic.MotionMagicAcceleration = Constants.Hood.ACCELERATION;

      // Soft limits
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Hood.MAX_POSE;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Hood.MIN_POSE;

      return config;
    }
  }

  // == Hardware ==============================================================
  private final TalonFX flywheelLeader;
  private final TalonFX flywheelFollower;
  private final TalonFXS hoodMotor;

  // == Control Requests =====================================================
 private final MotionMagicVelocityVoltage flywheelVelocityRequest = new MotionMagicVelocityVoltage(0.0).withEnableFOC(false);
private final MotionMagicVoltage hoodPositionRequest = new MotionMagicVoltage(0.0).withEnableFOC(false);

  // Flywheel follower was mechanically flipped again, so it must oppose the
  // leader's shaft rotation while still producing the same flywheel surface
  // direction at the wheel.
  private final Follower flywheelFollowerRequest =
      new Follower(
          Constants.Flywheel.FLYWHEEL_LEFT_MOTOR_ID,
          Constants.Flywheel.FollowerConfig.FOLLOWER_ALIGNMENT);

  // == Status Signals ====================================================
  private final StatusSignal<AngularVelocity> flywheelLeaderVelocity;
  private final StatusSignal<Voltage> flywheelLeaderVoltage;
  private final StatusSignal<Temperature> flywheelLeaderTempCelsius;
  private final StatusSignal<Temperature> flywheelFollowerTempCelsius;
  private final StatusSignal<?> hoodPosition;

  // == Constructor =======================================================
  public ShooterIOHardware() {
    flywheelLeader = new TalonFX(Constants.Flywheel.FLYWHEEL_LEFT_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelFollower = new TalonFX(Constants.Flywheel.FLYWHEEL_RIGHT_MOTOR_ID, Constants.RIO_CANBUS);
    hoodMotor = new TalonFXS(Constants.Hood.HOOD_MOTOR_ID, Constants.RIO_CANBUS);

    // Apply configs with retry logic - bare apply() can fail silently on RIO CAN
    // bus if the device is still booting. PhoenixUtil retries up to 5 times and
    // prints a DriverStation warning if all attempts fail.
    PhoenixUtil.applyConfig("Flywheel Leader",
        () -> flywheelLeader.getConfigurator().apply(FlywheelConfig.leader()));
    PhoenixUtil.applyConfig("Flywheel Follower",
        () -> flywheelFollower.getConfigurator().apply(FlywheelConfig.follower()));
    PhoenixUtil.applyConfig("Hood",
        () -> hoodMotor.getConfigurator().apply(HoodConfig.hood()));

    // Cache status signal references
    flywheelLeaderVelocity = flywheelLeader.getVelocity();
    flywheelLeaderVoltage = flywheelLeader.getMotorVoltage();
    flywheelLeaderTempCelsius = flywheelLeader.getDeviceTemp();
    flywheelFollowerTempCelsius = flywheelFollower.getDeviceTemp();
    hoodPosition = hoodMotor.getPosition();

    // Disable all status frames not explicitly re-enabled below.
    // optimizeBusUtilization() suppresses ALL frames - setUpdateFrequency calls
    // MUST come AFTER this call or they will be cleared.
    flywheelLeader.optimizeBusUtilization();
    flywheelFollower.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        flywheelLeader.getDutyCycle(),
        flywheelLeaderVoltage);

    // 50Hz - control loop needs fresh velocity and position every cycle.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelLeaderVelocity,
        hoodPosition);

    // 10Hz - temperature health check
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        flywheelLeaderTempCelsius,
        flywheelFollowerTempCelsius);

    // Followers MUST be set AFTER optimizeBusUtilization() - aggressive frame
    // disabling can break the follower control link if set before.
    flywheelFollower.setControl(flywheelFollowerRequest);

    // Initialize hood to known zero position
    hoodMotor.setPosition(Constants.Hood.ENCODER_ZERO_POSITION);
  }

  // == IO Implementation ==================================================
  @Override
  public void updateSlowInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(flywheelLeaderTempCelsius, flywheelFollowerTempCelsius);

    inputs.flywheelMaxTempCelsius = Math.max(
        flywheelLeaderTempCelsius.getValueAsDouble(),
        flywheelFollowerTempCelsius.getValueAsDouble());
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(flywheelLeaderVelocity, flywheelLeaderVoltage, hoodPosition);

    double motorRPS = flywheelLeaderVelocity.getValueAsDouble();
    inputs.flywheelLeaderMotorRPS = motorRPS;
    inputs.flywheelLeaderMotorRPM = rpsToRPM(motorRPS);
    inputs.flywheelAppliedVolts = flywheelLeaderVoltage.getValueAsDouble();
    inputs.hoodPositionRotations = hoodPosition.getValueAsDouble();
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelLeader.setControl(flywheelVelocityRequest.withVelocity(rpmToRPS(rpm)));
  }

  @Override
  public void stopFlywheels() {
    // Stop leader only - follower mirrors the leader's NeutralOut automatically.
    // Calling stopMotor() on the follower directly would break the Follower control
    // link (any direct command overrides it), requiring a re-establish on the next
    // velocity command. Leave the follower alone and let it follow to neutral.
    flywheelLeader.stopMotor();
  }

  @Override
  public void setHoodPose(double rawPosition) {
    hoodMotor.setControl(hoodPositionRequest.withPosition(rawPosition));
  }

  @Override
  public void stop() {
    // Stop leader only - follower mirrors to neutral automatically (see stopFlywheels).
    flywheelLeader.stopMotor();
    hoodMotor.stopMotor();
  }

  // == Unit Conversions ===================================================
  private double rpsToRPM(double rps) {
    return rps * 60.0;
  }

  private double rpmToRPS(double rpm) {
    return rpm / 60.0;
  }
}
