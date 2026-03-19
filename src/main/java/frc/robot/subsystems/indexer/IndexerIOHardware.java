package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

/**
 * IndexerIOHardware - Real hardware implementation of IndexerIO.
 *
 * Owns all hardware objects and translates them into the IndexerIOInputs
 * data structure read by IndexerSubsystem.
 *
 * CANrange detection strategy:
 * We configure any ToF sensor with a ProximityThreshold and read getIsDetected()
 * as a cached StatusSignal. This means the sensor itself decides "detected or not"
 * based on the configured threshold — we don't do the comparison in code.
 * getDistance() is also cached so students can watch raw values in AdvantageScope
 * while tuning the threshold in Phoenix Tuner X.
 *
 * All apply() calls use PhoenixUtil.applyConfig() for retry logic — a single
 * apply() on RIO CAN bus can return OK prematurely if the device is still booting.
 */
public class IndexerIOHardware implements IndexerIO {

    // == Motor Configuration =============================================

    private static TalonFXConfiguration conveyorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit       = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage =  12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }

    private static TalonFXConfiguration indexerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit       = 45.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage =  12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }

    private static CANrangeConfiguration chuteCANrangeConfig() {
        CANrangeConfiguration config = new CANrangeConfiguration();

        config.ProximityParams.ProximityThreshold = Constants.Indexer.FUEL_DETECTION_DISTANCE;
        config.ProximityParams.ProximityHysteresis = 0.025; // meters — TODO: Tune

        config.FovParams.FOVRangeX = 6.75; // degrees — TODO: Tune
        config.FovParams.FOVRangeY = 6.75; // degrees — TODO: Tune

        return config;
    }

    // == Hardware =================================================================
    private final TalonFX conveyorMotor;
    private final TalonFX indexerMotor;
    private final CANrange chuteToF;

    // == Control Requests ==========================================================
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut indexerVoltageRequest  = new VoltageOut(0.0);

    // == Status Signals ===========================================================
    private final StatusSignal<?> conveyorVelocity;
    private final StatusSignal<?> conveyorCurrent;
    private final StatusSignal<?> indexerVelocity;
    private final StatusSignal<?> indexerCurrent;
    private final StatusSignal<?> chuteDistance;
    private final StatusSignal<Boolean> chuteIsDetected;

    // == Constructor =============================================================
    public IndexerIOHardware() {
        conveyorMotor = new TalonFX(Constants.Indexer.CONVEYOR_MOTOR_ID, Constants.RIO_CANBUS);
        indexerMotorLeft  = new TalonFX(Constants.Indexer.KICKER_LEFT_MOTOR_ID,   Constants.RIO_CANBUS);
        indexerMotorRight  = new TalonFX(Constants.Indexer.KICKER_RIGHT_MOTOR_ID,   Constants.RIO_CANBUS);
        chuteToF      = new CANrange(Constants.Indexer.CHUTE_TOF_ID,       Constants.RIO_CANBUS);

        // Apply configs with retry logic — replaces the single-attempt local helper.
        // Five retries handles devices still booting when apply() is first called.
        PhoenixUtil.applyConfig("Conveyor",  () -> conveyorMotor.getConfigurator().apply(conveyorConfig()));
        PhoenixUtil.applyConfig("Indexer",   () -> indexerMotor.getConfigurator().apply(indexerConfig()));
        PhoenixUtil.applyConfig("Chute ToF", () -> chuteToF.getConfigurator().apply(chuteCANrangeConfig()));

        conveyorVelocity = conveyorMotor.getVelocity();
        conveyorCurrent  = conveyorMotor.getSupplyCurrent();
        indexerVelocity  = indexerMotor.getVelocity();
        indexerCurrent   = indexerMotor.getSupplyCurrent();
        chuteDistance    = chuteToF.getDistance();
        chuteIsDetected  = chuteToF.getIsDetected();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            conveyorVelocity, conveyorCurrent,
            indexerVelocity,  indexerCurrent,
            chuteDistance,    chuteIsDetected
        );

        conveyorMotor.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
        chuteToF.optimizeBusUtilization();
    }

    // == IO Implementation ========================================================
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            conveyorVelocity, conveyorCurrent,
            indexerVelocity,  indexerCurrent,
            chuteDistance,    chuteIsDetected
        );

        inputs.conveyorVelocityRPS = conveyorVelocity.getValueAsDouble();
        inputs.conveyorCurrentAmps = conveyorCurrent.getValueAsDouble();
        inputs.indexerVelocityRPS  = indexerVelocity.getValueAsDouble();
        inputs.indexerCurrentAmps  = indexerCurrent.getValueAsDouble();
        inputs.chuteDistanceMeters = chuteDistance.getValueAsDouble();
        inputs.chuteDetected       = chuteIsDetected.getValue();
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