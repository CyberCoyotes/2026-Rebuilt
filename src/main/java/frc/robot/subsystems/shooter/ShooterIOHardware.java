package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

/**
 * ShooterIOHardware - Real hardware implementation for the shooter subsystem.
 *
 * CAN OPTIMIZATION NOTES:
 * - Status signals are split into fast (50Hz) and slow (10Hz) groups
 * - updateInputs() only refreshes the 3 control-critical signals every cycle
 * - updateSlowInputs() refreshes the 4 diagnostic signals at 10Hz
 * - optimizeBusUtilization() disables all unneeded status frames
 * - Followers are set AFTER optimizeBusUtilization() to preserve control link
 *
 * @see Constants.Shooter for hardware configuration
 */
public class ShooterIOHardware implements ShooterIO {

  // ── Flywheel Configuration ─────────────────────────────────────────────────
  private static class FlywheelConfig {

    static TalonFXConfiguration competition() {
      TalonFXConfiguration config = base();

      // config.CurrentLimits.SupplyCurrentLimit = 45.0;
      // config.CurrentLimits.SupplyCurrentLimitEnable = true;
      // config.CurrentLimits.StatorCurrentLimit = 90.0;
      // config.CurrentLimits.StatorCurrentLimitEnable = true;
      // config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;
      // config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;

      return config;
    }

    static TalonFXConfiguration test() {
      TalonFXConfiguration config = base();

      config.CurrentLimits.SupplyCurrentLimit = 60.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 120.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

      return config;
    }

    private static TalonFXConfiguration base() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // Velocity closed loop — Slot 0
      config.Slot0.kP = 0.12; // TODO: Tune proportional gain based on flywheel velocity response
      config.Slot0.kV = 0.10; // TODO: Tune feedforward gain based on flywheel velocity response

      return config;
    }
  }

  // ── Hood Configuration ─────────────────────────────────────────────────────
  private static class HoodConfig {

    static TalonFXSConfiguration hood() {
      TalonFXSConfiguration config = new TalonFXSConfiguration();

      // Minion motor connected via JST connector
      config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      /*
       * Voltage limits — capped for safe hood movement during testing.
       * TODO: Increase after hood travel range is verified. 4V has been safe.
       */
      config.Voltage.PeakForwardVoltage = 4.0;
      config.Voltage.PeakReverseVoltage = -4.0;

      config.CurrentLimits.SupplyCurrentLimit = 30.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      // Position PID — Slot 0
      config.Slot0.kP = 1.0; // TODO: Tune hood position P
      config.Slot0.kI = 0.75; // TODO: Tune hood position I
      config.Slot0.kD = 0.0; // TODO: Tune hood position D

      // Soft limits — enable after travel range is confirmed
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9.15; // raw units
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

      return config;
    }
  }

  // ── Hardware ───────────────────────────────────────────────────────────────
  private final TalonFX flywheelMotorA;
  private final TalonFX flywheelMotorB;
  private final TalonFX flywheelMotorC;
  private final TalonFXS hoodMotor;
  private final CANcoder hoodEncoder;

  // ── Control Requests ───────────────────────────────────────────────────────
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);

  // ── Status Signals — Fast (50Hz, control-critical) ─────────────────────────
  private final StatusSignal<?> flywheelAVelocity;
  private final StatusSignal<?> flywheelAVoltage;
  private final StatusSignal<?> hoodPosition;

  // ── Status Signals — Slow (10Hz, diagnostics) ──────────────────────────────
  private final StatusSignal<?> flywheelACurrent;
  private final StatusSignal<?> hoodVoltage;
  private final StatusSignal<?> hoodCurrent;
  private final StatusSignal<?> hoodEncoderAbsPosition;

  public ShooterIOHardware() {
    flywheelMotorA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelMotorB = new TalonFX(Constants.Shooter.FLYWHEEL_B_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelMotorC = new TalonFX(Constants.Shooter.FLYWHEEL_C_MOTOR_ID, Constants.RIO_CANBUS);
    hoodMotor = new TalonFXS(Constants.Shooter.HOOD_MOTOR_ID, Constants.RIO_CANBUS);
    hoodEncoder = new CANcoder(Constants.Shooter.HOOD_POSE_ENCODER_ID, Constants.RIO_CANBUS);

    // Through Bore CANcoder config
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.0;
    hoodEncoder.getConfigurator().apply(encoderConfig);

    // Apply motor configs
    flywheelMotorA.getConfigurator().apply(FlywheelConfig.test());
    flywheelMotorB.getConfigurator().apply(FlywheelConfig.test());
    flywheelMotorC.getConfigurator().apply(FlywheelConfig.test());
    hoodMotor.getConfigurator().apply(HoodConfig.hood());

    // Cache status signal references
    flywheelAVelocity = flywheelMotorA.getVelocity();
    flywheelAVoltage = flywheelMotorA.getMotorVoltage();
    flywheelACurrent = flywheelMotorA.getSupplyCurrent();
    hoodPosition = hoodMotor.getPosition();
    hoodVoltage = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getSupplyCurrent();
    hoodEncoderAbsPosition = hoodEncoder.getAbsolutePosition();

    // 50Hz — control-critical signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelAVelocity,
        flywheelAVoltage,
        hoodPosition);

    // 10Hz — diagnostic signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        flywheelACurrent,
        hoodVoltage,
        hoodCurrent,
        hoodEncoderAbsPosition);

    // Disable all status frames not explicitly set above
    flywheelMotorA.optimizeBusUtilization();
    flywheelMotorB.optimizeBusUtilization();
    flywheelMotorC.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    hoodEncoder.optimizeBusUtilization();

    /*
     * Followers MUST be set AFTER optimizeBusUtilization()
     * otherwise aggressive frame disabling can break the follower control link
     */
    flywheelMotorB.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));
    flywheelMotorC.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));

    // Initialize hood to known zero position
    hoodMotor.setPosition(0.0);
  }

  /**
   * Refreshes control-critical signals only (3 signals at 50Hz).
   * Called every cycle (20ms) by ShooterSubsystem.periodic().
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // BaseStatusSignal.refreshAll(
    // flywheelAVelocity,
    // flywheelAVoltage,
    // hoodPosition
    // );

    // Flywheel — A is leader, B/C follow, so reading A is sufficient
    double motorRPS = flywheelAVelocity.getValueAsDouble();
    inputs.flywheelLeaderMotorRPS = motorRPS;
    inputs.flywheelLeaderMotorRPM = rpsToRPM(motorRPS);
    inputs.flywheelAppliedVolts = flywheelAVoltage.getValueAsDouble();

    // Hood position — always fresh for closed-loop control
    inputs.hoodPositionRotations = hoodPosition.getValueAsDouble();
  }

  /**
   * Refreshes diagnostic signals (4 signals at 10Hz).
   * Called every 5th cycle (100ms) by ShooterSubsystem.periodic().
   */
  @Override
  public void updateSlowInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        flywheelACurrent,
        hoodVoltage,
        hoodCurrent,
        hoodEncoderAbsPosition);

    inputs.flywheelCurrentAmps = flywheelACurrent.getValueAsDouble();
    inputs.hoodAppliedVolts = hoodVoltage.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();

    inputs.hoodAngleDegrees = inputs.hoodPositionRotations * 360.0; // only valid if 1 rot == 1 hood rev (verify with
                                                                    // your mechanism ratio)

    // Through Bore encoder
    inputs.hoodThroughBorePositionRotations = hoodEncoderAbsPosition.getValueAsDouble();
    inputs.hoodThroughBorePositionDegrees = inputs.hoodThroughBorePositionRotations * 360.0;
    inputs.hoodThroughBoreConnected = hoodEncoderAbsPosition.getStatus() == StatusCode.OK;
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    double motorRPS = rpmToRPS(rpm);
    flywheelMotorA.setControl(flywheelVelocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void stopFlywheels() {
    flywheelMotorA.stopMotor(); // followers stop automatically
  }

  @Override
  public void setHoodPose(double rawPosition) {
    hoodMotor.setControl(hoodPositionRequest.withPosition(rawPosition));
  }

  @Override
  public void setHoodVoltage(double volts) {
    hoodMotor.setControl(hoodVoltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    flywheelMotorA.stopMotor();
    hoodMotor.stopMotor();
  }

  // === Unit Conversions
  // ===============================================================
  private double rpsToRPM(double rps) {
    return rps * 60.0;
  }

  private double rpmToRPS(double rpm) {
    return rpm / 60.0;
  }
}