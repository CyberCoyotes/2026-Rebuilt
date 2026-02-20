package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class FlywheelConfig {
  private FlywheelConfig() {}

  public static TalonFXConfiguration competition() {
    TalonFXConfiguration config = base();

    // // ===== Current Limits (Competition Safe Start) =====
    // config.CurrentLimits.SupplyCurrentLimit = 45.0;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // config.CurrentLimits.StatorCurrentLimit = 90.0;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;

    // // ===== Ramps =====
    // config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;
    // config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0;

    return config;
  }

  public static TalonFXConfiguration test() {
    TalonFXConfiguration config = base();

    // // ===== Current Limits (Aggressive for Testing) =====
    // config.CurrentLimits.SupplyCurrentLimit = 60.0;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // config.CurrentLimits.StatorCurrentLimit = 120.0;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;

    // // ===== Ramps =====
    // config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
    // config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

    return config;
  }

  private static TalonFXConfiguration base() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // ===== Motor Output =====
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // ===== Closed Loop Gains (Velocity Slot 0) =====
    config.Slot0.kP = 0.12;
    // config.Slot0.kI = 0.0;
    // config.Slot0.kD = 0.0;
    //config.Slot0.kV = 0.12;

    // // ===== Voltage Limits =====
    // config.Voltage.PeakForwardVoltage = 12.0;
    // config.Voltage.PeakReverseVoltage = -12.0;

    return config;
  }
}