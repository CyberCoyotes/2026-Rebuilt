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

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC; // Testing

import frc.robot.Constants;

/**
 * ShooterIOHardware - Real hardware implementation for the shooter subsystem.
 *
 * CAN OPTIMIZATION NOTES:
 * - Only control-critical signals are read every cycle (50Hz)
 * - Current, voltage, and other diagnostics are captured by CTRE Hoot logs
 * - Through Bore encoder health is checked at 10Hz for runtime safety
 * - optimizeBusUtilization() disables all unneeded status frames
 * - Followers are set AFTER optimizeBusUtilization() to preserve control link
 *
 * @see Constants.Shooter for hardware configuration
 */
public class ShooterIOHardware implements ShooterIO {

  // ── Flywheel Configuration ─────────────────────────────────────────────────
  private static class FlywheelConfig {

    static TalonFXConfiguration competition() {
      TalonFXConfiguration config = leader();
      return config;
    }

    /** Config for Motor A (leader) — runs velocity closed-loop with PID and ramp. */
    // TODO: Dial in competition current limits and ramps
    static TalonFXConfiguration test() {
      TalonFXConfiguration config = leader();

      config.CurrentLimits.SupplyCurrentLimit = 60.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 120.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

      return config;
    }

    /**
     * Config for Motors B and C (followers).
     * Followers must NOT have a ramp period — the leader owns the ramp.
     * A ramp on the follower causes it to lag behind A, producing desync,
     * orange LEDs, and the characteristic whoop-whoop-whoop noise.
     * PID gains are also cleared — followers never run closed-loop independently.
     */
    static TalonFXConfiguration follower() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      config.CurrentLimits.SupplyCurrentLimit = 60.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 120.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      // No ramp period — leader controls the ramp, followers just mirror output
      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

      // No PID gains — followers never run closed-loop
      config.Slot0.kP = 0.0;
      config.Slot0.kV = 0.0;

      return config;
    }

    private static TalonFXConfiguration leader() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // Slot 0 — VelocityVoltage gains (kP in Volts/RPS)
      config.Slot0.kP = 0.12; // TODO: Tune
      config.Slot0.kV = 0.12; // TODO: Tune

      // Slot 1 — VelocityTorqueCurrentFOC gains (kP in Amps/RPS — different units, retune separately)
      config.Slot1.kP = 5.0;  // TODO: Tune — starting point for torque current; units are Amps not Volts
      config.Slot1.kV = 0.12; // TODO: Tune

      return config;
    }
  }

  // ── Hood Configuration ─────────────────────────────────────────────────────
  private static class HoodConfig {

    static TalonFXSConfiguration hood() {
      TalonFXSConfiguration config = new TalonFXSConfiguration();

      config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      /* Voltage limits — capped for safe hood movement during testing.
       * Consider increasing after hood travel range is verified. 4V has been safe without breakage. */
      config.Voltage.PeakForwardVoltage = 4.0;
      config.Voltage.PeakReverseVoltage = -4.0;

      config.CurrentLimits.SupplyCurrentLimit = 30.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      // Position PID — Slot 0
      config.Slot0.kP = 1.0;  // TODO: Tune
      config.Slot0.kI = 0.75; // TODO: Tune
      config.Slot0.kD = 0.0;

      // Soft limits — enable after travel range is confirmed
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9.15; // raw units
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

      return config;
    }
  }

  // === Hardware ===== 
  private final TalonFX flywheelMotorA;
  private final TalonFX flywheelMotorB;
  private final TalonFX flywheelMotorC;
  private final TalonFXS hoodMotor;
  private final CANcoder hoodEncoder;

  // === Control Requests =====
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);

  // TODO: Test VelocityTorqueCurrentFOC on flywheel — compare to VelocityVoltage with FOC, see if it improves acceleration or stability. 
  // Requires CAN FD (CANivore bus) for proper performance, but can be tested on RIO CAN for comparison purposes before flywheel motors are moved.
  private final VelocityTorqueCurrentFOC flywheelTorqueRequest = new VelocityTorqueCurrentFOC(0.0);
  
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0.0);
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);


  // === Status Signals =====
  private final StatusSignal<?> flywheelAVelocity;
  private final StatusSignal<?> hoodPosition;

  // === Status Signals =====
  // Current/voltage diagnostics are handled by CTRE Hoot logs, not read here.
  private final StatusSignal<?> hoodEncoderAbsPosition;

  public ShooterIOHardware() {
    flywheelMotorA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelMotorB = new TalonFX(Constants.Shooter.FLYWHEEL_B_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelMotorC = new TalonFX(Constants.Shooter.FLYWHEEL_C_MOTOR_ID, Constants.RIO_CANBUS);
    hoodMotor      = new TalonFXS(Constants.Shooter.HOOD_MOTOR_ID, Constants.RIO_CANBUS);
    hoodEncoder    = new CANcoder(Constants.Shooter.HOOD_POSE_ENCODER_ID, Constants.RIO_CANBUS);

    // Through Bore CANcoder config
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.0;
    hoodEncoder.getConfigurator().apply(encoderConfig);

    // Apply motor configs — B and C use a separate follower config (no ramp, no PID)
    flywheelMotorA.getConfigurator().apply(FlywheelConfig.test());
    flywheelMotorB.getConfigurator().apply(FlywheelConfig.follower());
    flywheelMotorC.getConfigurator().apply(FlywheelConfig.follower());
    hoodMotor.getConfigurator().apply(HoodConfig.hood());

    // Cache status signal references
    flywheelAVelocity     = flywheelMotorA.getVelocity();
    hoodPosition          = hoodMotor.getPosition();
    hoodEncoderAbsPosition = hoodEncoder.getAbsolutePosition();

    // 50Hz — control loop needs fresh velocity and position every cycle
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelAVelocity,
        hoodPosition
    );

    // 10Hz — encoder health check; fast enough to catch a disconnect safely
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        hoodEncoderAbsPosition
    );

    // Disable all status frames not explicitly configured above
    flywheelMotorA.optimizeBusUtilization();
    flywheelMotorB.optimizeBusUtilization();
    flywheelMotorC.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    hoodEncoder.optimizeBusUtilization();

    /* optimizeBusUtilization() suppresses ALL status frames on Motor A, including
     * the DutyCycle and MotorVoltage output signals that B and C need to follow.
     * Without these signals, followers lose sync — causing orange LEDs and the
     * whump-whump noise. Explicitly re-enable them at 100Hz on the leader AFTER
     * optimizing, so followers always have a fresh output value to mirror. */
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        flywheelMotorA.getDutyCycle(),
        flywheelMotorA.getMotorVoltage()
    );

    /* Followers MUST be set AFTER optimizeBusUtilization()
     * otherwise aggressive frame disabling can break the follower control link.
     * All three motors are physically aligned in the same direction — use Aligned. */
    flywheelMotorB.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));
    flywheelMotorC.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));

    // Initialize hood to known zero position
    hoodMotor.setPosition(0.0);
  }

  /**
   * Refreshes all runtime-necessary signals every cycle.
   * Diagnostics (current, voltage) are captured automatically by CTRE Hoot.
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Flywheel — A is leader, B/C follow, so reading A is sufficient
    double motorRPS = flywheelAVelocity.getValueAsDouble();
    inputs.flywheelLeaderMotorRPS = motorRPS;
    inputs.flywheelLeaderMotorRPM = rpsToRPM(motorRPS);

    // Hood position — needed every cycle for closed-loop control
    inputs.hoodPositionRotations = hoodPosition.getValueAsDouble();

    // Encoder health — if this goes false mid-match, subsystem can fault safely
    inputs.hoodThroughBoreConnected = hoodEncoderAbsPosition.getStatus() == StatusCode.OK;
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    double motorRPS = rpmToRPS(rpm);
    flywheelMotorA.setControl(flywheelVelocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void stopFlywheels() {
    // Explicitly stop all three motors — Follower control on B/C does NOT
    // automatically stop when A receives a neutral/stop request in Phoenix 6.
    flywheelMotorA.stopMotor();
    flywheelMotorB.stopMotor();
    flywheelMotorC.stopMotor();
    // Re-establish follower after stopping so they're ready for the next shot.
    flywheelMotorB.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned)); // TODO I question if this is necessary — does the follower control link persist through a stopMotor() call? If not, this is required to prevent desync. If it does persist, this is redundant but harmless.
    flywheelMotorC.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));
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
    flywheelMotorB.stopMotor();
    flywheelMotorC.stopMotor();
    hoodMotor.stopMotor();
  }

  // == Unit Conversions =====
  private double rpsToRPM(double rps) {
    return rps * 60.0;
  }

  private double rpmToRPS(double rpm) {
    return rpm / 60.0;
  }

  @Override
  public void setFlywheelVelocityTorqueFOC(double rpm) {
      double motorRPS = rpmToRPS(rpm);
      // .withSlot(1) selects the TorqueCurrentFOC-specific PID gains
      flywheelMotorA.setControl(flywheelTorqueRequest.withVelocity(motorRPS).withSlot(1));
  }

}