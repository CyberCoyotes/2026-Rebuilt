package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class IndexerIOHardware implements IndexerIO {

    // ── Conveyor Configuration ─────────────────────────────────────────────────
    private static class ConveyorConfig {

        static TalonFXSConfiguration conveyor() {
            TalonFXSConfiguration config = new TalonFXSConfiguration();

            // Minion motor connected via JST connector
            config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            config.CurrentLimits.SupplyCurrentLimit = 40.0;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40.0;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            return config;
        }
    }

    // ── Indexer Configuration ──────────────────────────────────────────────────
    private static class IndexerConfig {

        static TalonFXConfiguration indexer() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            config.CurrentLimits.SupplyCurrentLimit = 40.0;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 50.0;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            return config;
        }
    }

    // ── Hardware ───────────────────────────────────────────────────────────────
    private final TalonFXS conveyorMotor;
    private final TalonFX indexerMotor;

    private final CANrange hopperAToF;
    private final CANrange hopperBToF;
    // private final CANrange hopperCToF;
    // private final CANrange indexerToF;

    // ── Control Requests ───────────────────────────────────────────────────────
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut indexerVoltageRequest  = new VoltageOut(0.0);

    // ── Status Signals — 50Hz (runtime-critical) ───────────────────────────────
    // Velocity and current are kept because jam detection uses them for live logic.
    // Applied volts are captured by Hoot and not needed at runtime.
    private final StatusSignal<?> conveyorVelocity;
    private final StatusSignal<?> conveyorCurrent;
    private final StatusSignal<?> indexerVelocity;
    private final StatusSignal<?> indexerCurrent;

    public IndexerIOHardware() {
        conveyorMotor = new TalonFXS(Constants.Indexer.CONVEYOR_MOTOR_ID, Constants.RIO_CANBUS);
        indexerMotor  = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID, Constants.RIO_CANBUS);

        conveyorMotor.getConfigurator().apply(ConveyorConfig.conveyor());
        indexerMotor.getConfigurator().apply(IndexerConfig.indexer());

        hopperAToF = new CANrange(Constants.Indexer.HOPPER_A_TOF_ID);
        hopperBToF = new CANrange(Constants.Indexer.HOPPER_B_TOF_ID);
        // hopperCToF = new CANrange(Constants.Indexer.HOPPER_C_TOF_ID);

        // Cache signal references
        conveyorVelocity = conveyorMotor.getVelocity();
        conveyorCurrent  = conveyorMotor.getSupplyCurrent();
        indexerVelocity  = indexerMotor.getVelocity();
        indexerCurrent   = indexerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            conveyorVelocity, conveyorCurrent,
            indexerVelocity, indexerCurrent
        );

        conveyorMotor.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Velocity and current — needed for jam detection logic at runtime
        inputs.conveyorVelocityRPS  = conveyorVelocity.getValueAsDouble();
        inputs.conveyorCurrentAmps  = conveyorCurrent.getValueAsDouble();
        inputs.indexerVelocityRPS   = indexerVelocity.getValueAsDouble();
        inputs.indexerCurrentAmps   = indexerCurrent.getValueAsDouble();

        // ToF sensor inputs — uncomment as sensors are added
        // inputs.hopperADetected = hopperAToF.getDistance().getValueAsDouble() < TOF_DETECTION_THRESHOLD_MM;
        // inputs.hopperBDetected = hopperBToF.getDistance().getValueAsDouble() < TOF_DETECTION_THRESHOLD_MM;
        // inputs.hopperCDetected = hopperCToF.getDistance().getValueAsDouble() < TOF_DETECTION_THRESHOLD_MM;
        // inputs.gamePieceDetected = indexerToF.getDistance().getValueAsDouble() < TOF_DETECTION_THRESHOLD_MM;
    }

    @Override
    public void setConveyorMotor(double volts) {
        conveyorMotor.setControl(conveyorVoltageRequest.withOutput(volts));
    }

    @Override
    public void setIndexerMotor(double volts) {
        indexerMotor.setControl(indexerVoltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        conveyorMotor.stopMotor();
        indexerMotor.stopMotor();
    }
}