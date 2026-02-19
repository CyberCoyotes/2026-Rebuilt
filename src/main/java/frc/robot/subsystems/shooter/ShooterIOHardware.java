package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.util.TalonFXConfigs;
import frc.robot.util.flywheelConfig_Test;

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

  // ===== Hardware =====
  private final TalonFX flywheelMotorA;
  private final TalonFX flywheelMotorB;
  private final TalonFX flywheelMotorC;
  private final TalonFXS hoodMotor;
  private final CANcoder hoodEncoder;

  // ===== Control Requests (pre-allocated, reused each cycle) =====
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(false);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);

  // ===== Status Signals — Fast (50Hz, control-critical) =====
  private final StatusSignal<?> flywheelAVelocity;
  private final StatusSignal<?> flywheelAVoltage;
  private final StatusSignal<?> hoodPosition;

  // ===== Status Signals — Slow (10Hz, diagnostics) =====
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

    // CANcoder config
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.0;
    hoodEncoder.getConfigurator().apply(encoderConfig);

    // Apply motor configs
    flywheelMotorA.getConfigurator().apply(FlywheelConfig.test());
    flywheelMotorB.getConfigurator().apply(FlywheelConfig.test());
    flywheelMotorC.getConfigurator().apply(FlywheelConfig.test());
    hoodMotor.getConfigurator().apply(TalonFXConfigs.hoodConfig());

    // Cache status signal references
    flywheelAVelocity = flywheelMotorA.getVelocity();
    flywheelAVoltage = flywheelMotorA.getMotorVoltage();
    flywheelACurrent = flywheelMotorA.getSupplyCurrent();
    hoodPosition = hoodMotor.getPosition();
    hoodVoltage = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getSupplyCurrent();
    hoodEncoderAbsPosition = hoodEncoder.getAbsolutePosition();

    // ===== CAN Update Rates =====
    // 50Hz (every 20ms) — control-critical signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelAVelocity,
        flywheelAVoltage,
        hoodPosition
    );

    // 10Hz (every 100ms) — diagnostic signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        flywheelACurrent,
        hoodVoltage,
        hoodCurrent,
        hoodEncoderAbsPosition
    );

    // Disable all status frames we didn't explicitly set above
    flywheelMotorA.optimizeBusUtilization();
    flywheelMotorB.optimizeBusUtilization();
    flywheelMotorC.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    hoodEncoder.optimizeBusUtilization();

    // Followers MUST be set AFTER optimizeBusUtilization() —
    // otherwise the aggressive frame disabling can break the follower control link
    flywheelMotorB.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));
    flywheelMotorC.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));

    // If hood is physically at "home" on boot, this seeds the position.
    hoodMotor.setPosition(0.0);

    // Warm up signals ONCE at startup
    BaseStatusSignal.refreshAll(
        flywheelAVelocity,
        flywheelAVoltage,
        hoodPosition,
        flywheelACurrent,
        hoodVoltage,
        hoodCurrent,
        hoodEncoderAbsPosition
    );

  }

  /**
   * Refreshes control-critical signals only (3 signals at 50Hz).
   * Called every cycle (20ms) by ShooterSubsystem.periodic().
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Batch-read the 3 fast signals in one CAN transaction
    BaseStatusSignal.refreshAll(
        flywheelAVelocity,
        flywheelAVoltage,
        hoodPosition
    );

    // Flywheel (A is leader; B/C follow, so reading A is enough)
    double motorRPS = flywheelAVelocity.getValueAsDouble();
    inputs.flywheelLeaderMotorRPS = motorRPS;
    inputs.flywheelLeaderMotorRPM = rpsToRPM(motorRPS);
    inputs.flywheelAppliedVolts = flywheelAVoltage.getValueAsDouble();

    // Hood position (always fresh for closed-loop)
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
        hoodEncoderAbsPosition
    );

    inputs.flywheelCurrentAmps = flywheelACurrent.getValueAsDouble();
    inputs.hoodAppliedVolts = hoodVoltage.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();

    double hoodRot = inputs.hoodPositionRotations;
    // FIXME if real degrees are needed, measure actual hood gear ratio and convert properly
    inputs.hoodAngleDegrees = hoodRot * 360.0; // only if 1 rot == 1 hood rev (usually not true)

    // ThroughBore encoder
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

  // ===== Unit Conversions =====
  private double rpsToRPM(double rps) {
    return rps * 60.0;
  }

  private double rpmToRPS(double rpm) {
    return rpm / 60.0;
  }
}