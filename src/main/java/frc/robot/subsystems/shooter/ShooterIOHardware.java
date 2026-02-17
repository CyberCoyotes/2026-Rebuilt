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

public class ShooterIOHardware implements ShooterIO {

  // ===== Hardware =====
  private final TalonFX flywheelMotorA;
  private final TalonFX flywheelMotorB;
  private final TalonFX flywheelMotorC;
  private final TalonFXS hoodMotor;
  private final CANcoder hoodEncoder;

  // ===== Control Requests =====
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);

  // ===== Status Signals =====
  private final StatusSignal<?> flywheelAVelocity;
  private final StatusSignal<?> hoodPosition;
  private final StatusSignal<?> hoodVoltage;
  private final StatusSignal<?> hoodCurrent;
  private final StatusSignal<?> hoodEncoderAbsPosition;

  // ===== Conversion Constants =====
//   private static final double FLYWHEEL_GEAR_RATIO = 1.5; // TODO measure actual

  public ShooterIOHardware() {
    flywheelMotorA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, Constants.RIO_CAN_BUS);
    flywheelMotorB = new TalonFX(Constants.Shooter.FLYWHEEL_B_MOTOR_ID, Constants.RIO_CAN_BUS);
    flywheelMotorC = new TalonFX(Constants.Shooter.FLYWHEEL_C_MOTOR_ID, Constants.RIO_CAN_BUS);
    hoodMotor = new TalonFXS(Constants.Shooter.HOOD_MOTOR_ID, Constants.RIO_CAN_BUS);
    hoodEncoder = new CANcoder(Constants.Shooter.HOOD_POSE_ENCODER_ID, Constants.RIO_CAN_BUS);

    // CANcoder config
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.0;
    hoodEncoder.getConfigurator().apply(encoderConfig);

    // Apply configs
    flywheelMotorA.getConfigurator().apply(TalonFXConfigs.flywheelConfig());
    flywheelMotorB.getConfigurator().apply(TalonFXConfigs.flywheelConfig());
    flywheelMotorC.getConfigurator().apply(TalonFXConfigs.flywheelConfig());
    hoodMotor.getConfigurator().apply(TalonFXConfigs.hoodConfig());

    // Followers (THIS IS THE IMPORTANT PART YOU WORRIED ABOUT)
    // Note: follower references the LEADER DEVICE ID on the SAME BUS.
    flywheelMotorB.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));
    flywheelMotorC.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));

    // Status signals (minimal set)
    flywheelAVelocity = flywheelMotorA.getVelocity();
    hoodPosition = hoodMotor.getPosition();
    hoodVoltage = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getSupplyCurrent();
    hoodEncoderAbsPosition = hoodEncoder.getAbsolutePosition();

    // ✅ Update rates (this matters a LOT for CAN stability)
    // 50 Hz = every 20ms for control-critical
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelAVelocity,
        hoodPosition
    );

    // 10 Hz = every 100ms for current (slow)
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        hoodVoltage,
        hoodCurrent,
        hoodEncoderAbsPosition
    );

    // Optimize bus utilization
    flywheelMotorA.optimizeBusUtilization();
    flywheelMotorB.optimizeBusUtilization();
    flywheelMotorC.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    hoodEncoder.optimizeBusUtilization();

    // If hood is physically at "home" on boot, this is fine. Otherwise remove.
    hoodMotor.setPosition(0.0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        flywheelAVelocity,
        hoodPosition,
        hoodVoltage, 
        hoodCurrent,
        hoodEncoderAbsPosition
    );

    // Flywheel (A is leader; B/C follow, so logging A is enough for control)
    double motorRPS = flywheelAVelocity.getValueAsDouble();
    double motorRPM = rpsToRPM(motorRPS);

    inputs.flywheelMotorRPS = motorRPS;
    inputs.flywheelLeaderMotorRPM = motorRPM;
    inputs.flywheelLeaderMotorRPS = motorRPS; // system speed = leader motor
    // inputs.flywheelWheelRPM = motorRPM / FLYWHEEL_GEAR_RATIO; // when you measure it



    double hoodRotate = hoodPosition.getValueAsDouble();
    // Hood (keep these minimal but useful)
    inputs.hoodPositionRotations = hoodRotate;
    // FIXME if real degrees are needed, measure actual hood gear ratio and convert properly instead of assuming 1:1
    inputs.hoodAngleDegrees = hoodRotate * 360.0; // only if 1 rot == 1 hood rev (usually not true)
    inputs.hoodAppliedVolts = hoodVoltage.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();

    // Through-bore
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

  private double rpsToRPM(double rps) {
    return rps * 60.0;
  }

  private double rpmToRPS(double rpm) {
    return rpm / 60.0;
  }
}
