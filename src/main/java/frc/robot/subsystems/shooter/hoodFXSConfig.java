package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class hoodFXSConfig {

     public static TalonFXSConfiguration hood() {
        var config = new TalonFXSConfiguration();

        // TalonFXS motor arrangement - Minion connected via JST
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

        /* Voltage limits - cap output for safe hood movement during testing 
        * TODO: Increase after hood travel is verified.
        * 4 has been safe for testing.*/
        config.Voltage.PeakForwardVoltage = 4.0;   
        config.Voltage.PeakReverseVoltage = -4.0;

        // Current limits - lower for hood
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Position PID - Slot 0
        config.Slot0.kP = 1.0; // TODO Tune hood position P
        config.Slot0.kI = 0.75; // TODO Tune hood position I
        config.Slot0.kD = 0.0; // TODO Tune hood position D

        // Soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;  // Enable after tuning
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9.15;  // Raw units
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;  // Enable after tuning
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        return config;
    }
    
}
