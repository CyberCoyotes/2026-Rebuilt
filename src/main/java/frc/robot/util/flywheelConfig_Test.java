package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class flywheelConfig_Test {

    public static TalonFXConfiguration flywheelConfig() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // ===== Motor Output =====
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // ===== Closed Loop Gains (Velocity Slot 0) =====
        config.Slot0.kP = 0.12;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        // ===== Current Limits (AGGRESSIVE FOR TESTING) =====
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // ===== Ramps =====
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

        // ===== Voltage Limits =====
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }
}
