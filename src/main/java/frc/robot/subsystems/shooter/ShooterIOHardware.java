package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

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

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Flywheel.VOLTAGE_CLOSED_LOOP_RAMP_PERIOD; // 0.10

      config.MotorOutput.NeutralMode = Constants.Flywheel.LeaderConfig.NEUTRAL_MODE;
      config.MotorOutput.Inverted = Constants.Flywheel.LeaderConfig.INVERTED;

      // Slot 0 — VelocityVoltage gains (kP in Volts/RPS)
      // Tuned 2026-03-01: kV=0.126 → ±30 RPM steady-state at 3300 RPM, 7V draw, no oscillation
      config.Slot0.kP = Constants.Flywheel.KP; // 0.15
      config.Slot0.kV = Constants.Flywheel.KV; // 0.119; // don't touch — well tuned
      config.Slot0.kD = Constants.Flywheel.KD; // 0.001;

      return config;
    }
  }

  // == Hood Configuration ==========================================================
  private static class HoodConfig {

    static TalonFXSConfiguration hood() {
      TalonFXSConfiguration config = new TalonFXSConfiguration();

      config.Commutation.MotorArrangement = Constants.Hood.Config.MOTOR_ARRANGEMENT;
      config.MotorOutput.NeutralMode = Constants.Hood.Config.NEUTRAL_MODE;
      config.MotorOutput.Inverted = Constants.Hood.Config.INVERTED;

      // Voltage limits — capped for safe hood movement and plenty fast for short-range repositioning.
      config.Voltage.PeakForwardVoltage = Constants.Hood.PEAK_FORWARD_VOLTAGE; // 4.0;
      config.Voltage.PeakReverseVoltage = Constants.Hood.PEAK_REVERSE_VOLTAGE; // -4.0;

      config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.SUPPLY_CURRENT_LIMIT; // 30.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      // Position PID — Slot 0
      // MotionMagicVoltage with kP/kD is sufficient for hood positioning.
      /* TODO: Current not using MotionMagic for Hood position, but ideally it should be. 
      * If we switch to MotionMagic, we need to run kP/kD for smooth, stable positioning without overshoot or oscillation.
      */
      config.Slot0.kP = Constants.Hood.KP; // 1.00
      config.Slot0.kI = Constants.Hood.KI; // 0.0
      config.Slot0.kD = Constants.Hood.KD; // 0.00
      config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.CRUISE_VELOCITY; // 10.0
      config.MotionMagic.MotionMagicAcceleration = Constants.Hood.ACCELERATION; // 20.0
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
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(false);

  // MotionMagicVoltage respects the cruise velocity/accel profile set in config,
  // limiting current spikes during large position changes.
  // PositionVoltage commands maximum effort immediately — do not use for hood.
  private final MotionMagicVoltage hoodPositionRequest = new MotionMagicVoltage(0.0);

  // Flywheel follower was mechanically flipped again, so it must oppose the
  // leader's shaft rotation while still producing the same flywheel surface
  // direction at the wheel.
  private final Follower flywheelFollowerRequest =
      new Follower(
          Constants.Flywheel.FLYWHEEL_LEFT_MOTOR_ID,
          Constants.Flywheel.FollowerConfig.FOLLOWER_ALIGNMENT);
      // new Follower(flywheelLeader.getDeviceID(), MotorAlignmentValue.Opposed); Alternative approach

  // == Status Signals ====================================================

  /* 
  * These Status Signals were not typed previously <?>, but trying Typed e.g. <AngularVelocity> 
  * example
  *   private final StatusSignal<?> flywheelLeaderVelocity;
  */ 
  private final StatusSignal<AngularVelocity> flywheelLeaderVelocity;
  private final StatusSignal<Voltage> flywheelLeaderVoltage;
  private final StatusSignal<Temperature> flywheelLeaderTempCelsius;
  private final StatusSignal<Temperature> flywheelFollowerTempCelsius;
  private final StatusSignal<?> hoodPosition;

  // == Constructor =======================================================
  public ShooterIOHardware() {
    flywheelLeader = new TalonFX(Constants.Flywheel.FLYWHEEL_LEFT_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelFollower = new TalonFX(Constants.Flywheel.FLYWHEEL_RIGHT_MOTOR_ID, Constants.RIO_CANBUS);
    hoodMotor      = new TalonFXS(Constants.Hood.HOOD_MOTOR_ID, Constants.RIO_CANBUS);

    // Apply configs with retry logic — bare apply() can fail silently on RIO CAN
    // bus if the device is still booting. PhoenixUtil retries up to 5 times and
    // prints a DriverStation warning if all attempts fail.
    PhoenixUtil.applyConfig("Flywheel Leader",
        () -> flywheelLeader.getConfigurator().apply(FlywheelConfig.leader()));
    PhoenixUtil.applyConfig("Flywheel Follower",
        () -> flywheelFollower.getConfigurator().apply(FlywheelConfig.follower()));
    PhoenixUtil.applyConfig("Hood",
        () -> hoodMotor.getConfigurator().apply(HoodConfig.hood()));

    // Cache status signal references
    flywheelLeaderVelocity     = flywheelLeader.getVelocity();
    flywheelLeaderVoltage = flywheelLeader.getMotorVoltage();
    flywheelLeaderTempCelsius  = flywheelLeader.getDeviceTemp();
    flywheelFollowerTempCelsius  = flywheelFollower.getDeviceTemp();
    hoodPosition          = hoodMotor.getPosition();

    // Disable all status frames not explicitly re-enabled below.
    // optimizeBusUtilization() suppresses ALL frames — setUpdateFrequency calls
    // MUST come AFTER this call or they will be cleared.
    flywheelLeader.optimizeBusUtilization();
    flywheelFollower.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        flywheelLeader.getDutyCycle(),
        flywheelLeaderVoltage
    );

    // 50Hz — control loop needs fresh velocity and position every cycle.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelLeaderVelocity,
        hoodPosition
    );

    // 10Hz — temperature health check
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        flywheelLeaderTempCelsius,
        flywheelFollowerTempCelsius
    );

    // Followers MUST be set AFTER optimizeBusUtilization() — aggressive frame
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
    inputs.flywheelAppliedVolts   = flywheelLeaderVoltage.getValueAsDouble();
    inputs.hoodPositionRotations  = hoodPosition.getValueAsDouble();
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelLeader.setControl(flywheelVelocityRequest.withVelocity(rpmToRPS(rpm)));
  }

  @Override
  public void stopFlywheels() {
    // Stop leader only — follower mirrors the leader's NeutralOut automatically.
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
    // Stop leader only — follower mirrors to neutral automatically (see stopFlywheels).
    flywheelLeader.stopMotor();
    hoodMotor.stopMotor();
  }

  // == Unit Conversions ===================================================
  private double rpsToRPM(double rps) { return rps * 60.0; }
  private double rpmToRPS(double rpm) { return rpm / 60.0; }
}
