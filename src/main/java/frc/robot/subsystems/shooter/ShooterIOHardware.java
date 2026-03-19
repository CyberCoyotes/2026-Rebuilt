package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    static TalonFXConfiguration leader() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimit = 45.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 90.0;
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

  // ── Hood Configuration ─────────────────────────────────────────────────────
  private static class HoodConfig {

    static TalonFXSConfiguration hood() {
      TalonFXSConfiguration config = new TalonFXSConfiguration();

      config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // Voltage limits — capped for safe hood movement
      config.Voltage.PeakForwardVoltage = 4.0;
      config.Voltage.PeakReverseVoltage = -4.0;

      config.CurrentLimits.SupplyCurrentLimit = 30.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      // Position PID — Slot 0
      // kI removed — integrator windup was causing current spikes during shooting cycles.
      // MotionMagicVoltage with kP/kD is sufficient for hood positioning.
      // TODO: Retune kP and add kD after kI removal
      config.Slot0.kP = 1.00;
      config.Slot0.kI = 0.0;  // Removed — was 0.75, caused integrator windup
      config.Slot0.kD = 0.00;

      // MotionMagic profile — controls velocity/accel during position moves
      // Limits current spike from large position changes vs raw PositionVoltage
      // TODO: Tune these values for smooth hood movement
      config.MotionMagic.MotionMagicCruiseVelocity = 10.0;
      config.MotionMagic.MotionMagicAcceleration = 20.0;

      // Soft limits
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.15;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

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

  // == Status Signals ====================================================
  private final StatusSignal<?> flywheelLeaderVelocity;
  private final StatusSignal<?> flywheelLeaderVoltage;
  private final StatusSignal<?> flywheelLeaderTempCelsius;
  private final StatusSignal<?> flywheelFollowerTempCelsius;
  private final StatusSignal<?> hoodPosition;

  // == Constructor =======================================================
  public ShooterIOHardware() {
    flywheelLeader = new TalonFX(Constants.Shooter.FLYWHEEL_LEFT_MOTOR_ID, Constants.RIO_CANBUS);
    flywheelFollower = new TalonFX(Constants.Shooter.FLYWHEEL_RIGHT_MOTOR_ID, Constants.RIO_CANBUS);
    hoodMotor      = new TalonFXS(Constants.Shooter.HOOD_MOTOR_ID, Constants.RIO_CANBUS);

    // Apply configs with retry logic — bare apply() can fail silently on RIO CAN
    // bus if the device is still booting. PhoenixUtil retries up to 5 times and
    // prints a DriverStation warning if all attempts fail.
    PhoenixUtil.applyConfig("Flywheel A leader",
        () -> flywheelLeader.getConfigurator().apply(FlywheelConfig.leader()));
    PhoenixUtil.applyConfig("Flywheel B follower",
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

    // 100Hz — DutyCycle and MotorVoltage must be re-enabled on Motor A so
    // followers B and C can mirror output. Without these, followers lose sync.
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
    flywheelFollower.setControl(new Follower(Constants.Shooter.FLYWHEEL_LEFT_MOTOR_ID, MotorAlignmentValue.Aligned));

    // Initialize hood to known zero position
    hoodMotor.setPosition(0.0);
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
    // Explicitly stop all three — Follower control on B/C does NOT automatically
    // stop when A receives a neutral request in Phoenix 6.
    flywheelLeader.stopMotor();
    flywheelFollower.stopMotor();

    // Re-establish follower after stopping — stopMotor() sends NeutralOut which
    // overrides the Follower request, so followers must be reset afterward.
    flywheelFollower.setControl(new Follower(Constants.Shooter.FLYWHEEL_LEFT_MOTOR_ID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void setHoodPose(double rawPosition) {
    hoodMotor.setControl(hoodPositionRequest.withPosition(rawPosition));
  }

  @Override
  public void stop() {
    flywheelLeader.stopMotor();
    flywheelFollower.stopMotor();
    hoodMotor.stopMotor();
  }

  // == Unit Conversions ===================================================
  private double rpsToRPM(double rps) { return rps * 60.0; }
  private double rpmToRPS(double rpm) { return rpm / 60.0; }
}