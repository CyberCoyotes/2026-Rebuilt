package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/**
 * IndexerIOHardware - Real hardware implementation of IndexerIO.
 *
 * Owns all hardware objects and translates them into the IndexerIOInputs
 * data structure read by IndexerSubsystem.
 *
 * CANrange detection strategy:
 * We configure each sensor with a ProximityThreshold and read getIsDetected()
 * as a cached StatusSignal. This means the sensor itself decides "detected or not"
 * based on the configured threshold — we don't do the comparison in code.
 * getDistance() is also cached so students can watch raw values in AdvantageScope
 * while tuning the threshold in Phoenix Tuner X.
 */
public class IndexerIOHardware implements IndexerIO {

    // == Detection Threshold ======
    /**
     * Distance in meters below which a CANrange reports "detected".
     * This value is pushed to both sensors via CANrangeConfiguration.
     * To tune: watch hopperA/BDistanceMeters in AdvantageScope with a game piece
     * at each sensor, then set the threshold to ~50% between the piece-present
     * and piece-absent readings.
     */

    private static final double TOF_DETECTION_THRESHOLD_METERS = 0.15; // ~6 inches


    // == Motor Configuration ======
    // Config builders are private static methods so students can find and tune
    // all motor settings in this file without hunting through Constants.java.

    private static TalonFXSConfiguration conveyorConfig() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        // Minion motor connected via JST connector
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit       = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }

    private static TalonFXConfiguration indexerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit       = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = 50.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        return config;
    }

private static CANrangeConfiguration hopperCANrangeConfig() {
    CANrangeConfiguration configHopper = new CANrangeConfiguration();

    // Sensor trips "detected" when distance drops below this value
    configHopper.ProximityParams.ProximityThreshold = TOF_DETECTION_THRESHOLD_METERS;

    // Prevents chattering when a piece sits right at the threshold.
    // Sensor won't un-detect until distance rises ~2.5cm above the threshold.
    configHopper.ProximityParams.ProximityHysteresis = 0.025; // meters — TODO: Tune

    // Narrow the FOV to reduce false positives from hopper walls.
    // Full range is ±6.75° each axis. Reduce if you see spurious detections.
    configHopper.FovParams.FOVRangeX = 6.75; // degrees — TODO: Tune down if needed
    configHopper.FovParams.FOVRangeY = 6.75; // degrees — TODO: Tune down if needed

    return configHopper;
}

private static CANrangeConfiguration chuteCANrangeConfig() {
    CANrangeConfiguration configChute = new CANrangeConfiguration();

    // Sensor trips "detected" when distance drops below this value
    configChute.ProximityParams.ProximityThreshold = Constants.Indexer.CHUTE_DETECTION_THRESHOLD_METERS;

    // Prevents chattering when a piece sits right at the threshold.
    // Sensor won't un-detect until distance rises ~2.5cm above the threshold.
    configChute.ProximityParams.ProximityHysteresis = 0.025; // meters — TODO: Tune

    // Narrow the FOV to reduce false positives from hopper walls.
    // Full range is ±6.75° each axis. Reduce if you see spurious detections.

    configChute.FovParams.FOVRangeX = 6.75; // degrees — TODO: Tune down if needed
    configChute.FovParams.FOVRangeY = 6.75; // degrees — TODO: Tune down if needed

    return configChute;
}
    // ── Hardware ───────────────────────────────────────────────────────────────
    private final TalonFXS conveyorMotor;
    private final TalonFX  indexerMotor;
    private final CANrange hopperAToF;
    private final CANrange hopperBToF;
    private final CANrange chuteToF;

    // ── Control Requests ───────────────────────────────────────────────────────
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut indexerVoltageRequest  = new VoltageOut(0.0);

    // ── Status Signals ─────────────────────────────────────────────────────────
    // Motors at 50 Hz — needed each cycle for jam detection logic
    private final StatusSignal<?> conveyorVelocity;
    private final StatusSignal<?> conveyorCurrent;
    private final StatusSignal<?> indexerVelocity;
    private final StatusSignal<?> indexerCurrent;

    // CANrange distance signals at 50 Hz — raw meters, logged for threshold tuning
    private final StatusSignal<?> hopperADistance;
    private final StatusSignal<?> hopperBDistance;
    private final StatusSignal<?> chuteDistance;

    // CANrange detection signals at 50 Hz — boolean result of onboard threshold compare
    private final StatusSignal<Boolean> hopperAIsDetected;
    private final StatusSignal<Boolean> hopperBIsDetected;
    private final StatusSignal<Boolean> chuteIsDetected;

    // ── Constructor ────────────────────────────────────────────────────────────
    public IndexerIOHardware() {
        conveyorMotor = new TalonFXS(Constants.Indexer.CONVEYOR_MOTOR_ID, Constants.RIO_CANBUS);
        indexerMotor  = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID, Constants.RIO_CANBUS);
        hopperAToF    = new CANrange(Constants.Indexer.HOPPER_A_TOF_ID, Constants.RIO_CANBUS);
        hopperBToF    = new CANrange(Constants.Indexer.HOPPER_B_TOF_ID, Constants.RIO_CANBUS);
        chuteToF      = new CANrange(Constants.Indexer.CHUTE_TOF_ID, Constants.RIO_CANBUS);

        // Apply configurations with error checking.
        // IMPORTANT: apply() can fail silently if the device is unreachable on CAN.
        // When that happens the device keeps its previous config, which may have wrong
        // current limits or direction. The warning below makes that visible.
        applyConfig("Conveyor",  () -> conveyorMotor.getConfigurator().apply(conveyorConfig()));
        applyConfig("Indexer",   () -> indexerMotor.getConfigurator().apply(indexerConfig()));
        applyConfig("HopperA ToF", () -> hopperAToF.getConfigurator().apply(hopperCANrangeConfig()));
        applyConfig("HopperB ToF", () -> hopperBToF.getConfigurator().apply(hopperCANrangeConfig()));
        applyConfig("Chute ToF", () -> chuteToF.getConfigurator().apply(chuteCANrangeConfig()));

        // Cache signal references after apply() so handles are clean
        conveyorVelocity   = conveyorMotor.getVelocity();
        conveyorCurrent    = conveyorMotor.getSupplyCurrent();
        indexerVelocity    = indexerMotor.getVelocity();
        indexerCurrent     = indexerMotor.getSupplyCurrent();
        hopperADistance    = hopperAToF.getDistance();
        hopperBDistance    = hopperBToF.getDistance();
        chuteDistance      = chuteToF.getDistance();
        hopperAIsDetected  = hopperAToF.getIsDetected();
        hopperBIsDetected  = hopperBToF.getIsDetected();
        chuteIsDetected    = chuteToF.getIsDetected();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            conveyorVelocity, conveyorCurrent,
            indexerVelocity,  indexerCurrent,
            hopperADistance,  hopperAIsDetected,
            hopperBDistance,  hopperBIsDetected,
            chuteDistance,    chuteIsDetected
        );

        conveyorMotor.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
        hopperAToF.optimizeBusUtilization();
        hopperBToF.optimizeBusUtilization();
        chuteToF.optimizeBusUtilization();
    }

    // ── Config Helper ──────────────────────────────────────────────────────────
    /**
     * Applies a Phoenix 6 configuration and reports a warning if it fails.
     *
     * @param deviceName Human-readable name for the warning message
     * @param applyCall  Lambda that performs the apply() call and returns StatusCode
     */
    private void applyConfig(String deviceName, java.util.function.Supplier<StatusCode> applyCall) {
        StatusCode result = applyCall.get();
        if (!result.isOK()) {
            DriverStation.reportWarning(
                "[IndexerIOHardware] " + deviceName + " config apply failed: " + result.getName(), false
            );
        }
    }

    // ── IO Implementation ──────────────────────────────────────────────────────
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Refresh all cached signals in a single CAN read
        BaseStatusSignal.refreshAll(
            conveyorVelocity, conveyorCurrent,
            indexerVelocity,  indexerCurrent,
            hopperADistance,  hopperAIsDetected,
            hopperBDistance,  hopperBIsDetected,
            chuteDistance,    chuteIsDetected
        );

        // Motor data
        inputs.conveyorVelocityRPS = conveyorVelocity.getValueAsDouble();
        inputs.conveyorCurrentAmps = conveyorCurrent.getValueAsDouble();
        inputs.indexerVelocityRPS  = indexerVelocity.getValueAsDouble();
        inputs.indexerCurrentAmps  = indexerCurrent.getValueAsDouble();

        // CANrange data — distance for logging, getIsDetected() for game piece logic
        inputs.hopperADistanceMeters = hopperADistance.getValueAsDouble();
        inputs.hopperADetected       = hopperAIsDetected.getValue();
        inputs.hopperBDistanceMeters = hopperBDistance.getValueAsDouble();
        inputs.hopperBDetected       = hopperBIsDetected.getValue();
        inputs.chuteDistanceMeters   = chuteDistance.getValueAsDouble();
        inputs.chuteDetected         = chuteIsDetected.getValue();
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