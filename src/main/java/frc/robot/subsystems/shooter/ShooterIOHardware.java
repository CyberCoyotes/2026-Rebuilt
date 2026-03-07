package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

      config.CurrentLimits.SupplyCurrentLimit = 45.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 90.0;
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

      // On 2/27 was set at 60
      config.CurrentLimits.SupplyCurrentLimit = 45.0; // TODO: Tune Flywheel supply current limit for testing.
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      // On 2/27 was set at 120
      config.CurrentLimits.StatorCurrentLimit = 90.0; // TODO: Tune Flywheel stator current limit for testing.
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.10;

      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // Slot 0 — VelocityVoltage gains (kP in Volts/RPS)
      // Tuned 2026-03-01: kV=0.126 → ±30 RPM steady-state at 3300 RPM, 7V draw, no oscillation
      config.Slot0.kP = 0.15;
      config.Slot0.kV = 0.119; // don't touch — well tuned
      config.Slot0.kD = 0.001;

      return config;
    }
  }

  // ==== Hood Configuration ======
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
      config.CurrentLimits.StatorCurrentLimit = 20.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.MotionMagic.MotionMagicCruiseVelocity = 3.0;  // rot/s — conservative start
      config.MotionMagic.MotionMagicAcceleration = 6.0;     // rot/s²

      // Previous Position PID — Slot 0
      // Tuned 2026-03-01: 
      // kP=1.0, kI=0.75 → no steady-state error at 9.15 rotations, acceptable oscillation, no overshoot
      // config.Slot0.kP = 1.00; 
      // config.Slot0.kI = 0.75;
      // config.Slot0.kD = 0.00;
      // config.Slot0.kP = 1.00;
      // config.Slot0.kI = 0.10; // Drop significantly — was winding up
      // config.Slot0.kD = 0.05; // Add some dampening for disturbances

      /* TODO: Tuning Sequence for the Hood
       * With MotionMagic in place and all gains at zero, your session order is:
       * 1. MotionMagic profile first — command hood to 9.15 rotations with all gains zero. It won't get there, but watch how the velocity profile shapes up in Tuner
       * 2. kP — increase in 0.1 increments until it reliably reaches target. Stop before oscillation
       * 3. kD — only if you see overshoot. Start at kP × 0.1
       * 4. kS — only if it stalls before moving. Bump 0.1 at a time
       * 5. kG — only if gravity is pulling the hood off position at rest
       *
       * Given your 4V cap and good gearing, guess is kP = 0.3 gets you 90% of the way there.
       */

      config.Slot0.kS = 0.0; // Add last
      config.Slot0.kG = 0.0; // Only if gravity matters
      config.Slot0.kP = 0.3; // Tune first
      config.Slot0.kD = 0.0; // Tune second
      config.Slot0.kI = 0.0; // Leave dead

      // Soft limits — enable after travel range is confirmed
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9.15; // Raw rotational units
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
  // private final CANcoder hoodEncoder;

  // === Control Requests =====`1
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0).withEnableFOC(false);
  
private final MotionMagicVoltage hoodPositionRequest = new MotionMagicVoltage(0.0);
  // private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);


  // === Status Signals =====
  private final StatusSignal<?> flywheelAVelocity;
  private final StatusSignal<?> flywheelAMotorVoltage; // applied volts — also re-enabled at 100Hz for follower sync
  private final StatusSignal<?> flywheelATempCelsius;
  private final StatusSignal<?> flywheelBTempCelsius;
  private final StatusSignal<?> flywheelCTempCelsius;
  private final StatusSignal<?> hoodPosition;

  public ShooterIOHardware() {
    flywheelMotorA = new TalonFX(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelMotorB = new TalonFX(Constants.Shooter.FLYWHEEL_B_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelMotorC = new TalonFX(Constants.Shooter.FLYWHEEL_C_MOTOR_ID, Constants.RIO_CANBUS);
    hoodMotor      = new TalonFXS(Constants.Shooter.HOOD_MOTOR_ID, Constants.RIO_CANBUS);

    // Apply motor configs — B and C use a separate follower config (no ramp, no PID)
    flywheelMotorA.getConfigurator().apply(FlywheelConfig.leader());
    flywheelMotorB.getConfigurator().apply(FlywheelConfig.follower());
    flywheelMotorC.getConfigurator().apply(FlywheelConfig.follower());
    hoodMotor.getConfigurator().apply(HoodConfig.hood());

    // Cache status signal references
    flywheelAVelocity      = flywheelMotorA.getVelocity();
    flywheelAMotorVoltage  = flywheelMotorA.getMotorVoltage();
    flywheelATempCelsius   = flywheelMotorA.getDeviceTemp();
    flywheelBTempCelsius   = flywheelMotorB.getDeviceTemp();
    flywheelCTempCelsius   = flywheelMotorC.getDeviceTemp();
    hoodPosition           = hoodMotor.getPosition();

    // Disable all status frames not explicitly configured below.
    // optimizeBusUtilization() suppresses ALL status frames — setUpdateFrequency
    // calls MUST come AFTER this call, otherwise they get cleared.
    flywheelMotorA.optimizeBusUtilization();
    flywheelMotorB.optimizeBusUtilization();
    flywheelMotorC.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();

    // 100Hz — DutyCycle and MotorVoltage must be re-enabled on Motor A so
    // followers B and C can mirror output. Without these, followers lose sync,
    // causing orange LEDs and the whump-whump noise.
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        flywheelMotorA.getDutyCycle(),
        flywheelAMotorVoltage
    );

    // 50Hz — control loop needs fresh velocity and position every cycle.
    // Must be set AFTER optimizeBusUtilization() or it will be cleared.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        flywheelAVelocity,
        hoodPosition
    );

    // 10Hz — encoder health check; fast enough to catch a disconnect safely.
    // Must be set AFTER optimizeBusUtilization() or it will be cleared.
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        flywheelATempCelsius,
        flywheelBTempCelsius,
        flywheelCTempCelsius
    );

    /* Followers MUST be set AFTER optimizeBusUtilization()
     * otherwise aggressive frame disabling can break the follower control link.
     * All three motors are physically aligned in the same direction — use Aligned. */
    flywheelMotorB.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));
    flywheelMotorC.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));

    // Initialize hood to known zero position
    hoodMotor.setPosition(0.0);
  }

  @Override
  public void updateSlowInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(flywheelATempCelsius, flywheelBTempCelsius, flywheelCTempCelsius);

    inputs.flywheelMaxTempCelsius = Math.max(
        flywheelATempCelsius.getValueAsDouble(),
        Math.max(flywheelBTempCelsius.getValueAsDouble(), flywheelCTempCelsius.getValueAsDouble()));
  }

  /**
   * Refreshes all runtime-necessary signals every cycle.
   * Diagnostics (current, voltage) are captured automatically by CTRE Hoot.
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(flywheelAVelocity, flywheelAMotorVoltage, hoodPosition);

    // Flywheel — A is leader, B/C follow, so reading A is sufficient
    double motorRPS = flywheelAVelocity.getValueAsDouble();
    inputs.flywheelLeaderMotorRPS = motorRPS;
    inputs.flywheelLeaderMotorRPM = rpsToRPM(motorRPS);
    inputs.flywheelAppliedVolts   = flywheelAMotorVoltage.getValueAsDouble();

    // Hood position — needed every cycle for closed-loop control
    inputs.hoodPositionRotations = hoodPosition.getValueAsDouble();

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
    // stopMotor() send NeutralOut which overrides the Follower request, 
    // so we need to reset the followers after stopping.
    flywheelMotorB.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned)); 
    flywheelMotorC.setControl(new Follower(Constants.Shooter.FLYWHEEL_A_MOTOR_ID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void setHoodPose(double rawPosition) {
    hoodMotor.setControl(hoodPositionRequest.withPosition(rawPosition));
  }

  // @Override
  // public void setHoodVoltage(double volts) {
  //   hoodMotor.setControl(hoodVoltageRequest.withOutput(volts));
  // }

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

}
