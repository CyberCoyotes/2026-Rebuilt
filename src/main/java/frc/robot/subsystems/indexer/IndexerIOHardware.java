package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants;
import frc.robot.util.TalonFXConfigs;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.signals.MeasurementHealthValue;



/**
 * IndexerIOHardware - Real hardware implementation for the indexer subsystem.
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Optimized status signal updates for performance
 * - ToF sensors configured for short-range detection
 *
 * Motor naming convention:
 * - "conveyor" motor moves pieces along hopper conveyor
 * - "indexer" motor feeds pieces to shooter
 *
 * @see Constants.Indexer for hardware configuration
 * @see IndexerSubsystemBasic for a simpler direct-hardware approach (good for learning)
 */

public class IndexerIOHardware implements IndexerIO {

    // ===== Hardware =====
    private final TalonFXS conveyorMotor;   // TalonFXS with Minion motor
    private final TalonFX indexerMotor;  // Also called "indexer" in Constants

    // Time-of-Flight sensors
    // private final CANrange indexerToF;   // Detects game piece ready to shoot
    private final CANrange hopperAToF; 
    private final CANrange hopperBToF;
    // private final CANrange hopperCToF;

    // ===== Control Requests =====
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut indexerVoltageRequest = new VoltageOut(0.0);

    // ===== Status Signals (for efficient reading) =====
    // Note: Phoenix 6 uses typed StatusSignals, we just read the value
    private final StatusSignal<?> conveyorVelocity;
    private final StatusSignal<?> conveyorAppliedVolts;
    private final StatusSignal<?> conveyorCurrent;
    private final StatusSignal<?> indexerVelocity;
    private final StatusSignal<?> indexerAppliedVolts;
    private final StatusSignal<?> indexerCurrent;

    // ===== ToF (CANrange) Status Signals =====
    private final StatusSignal<?> hopperADistance;     // meters
    private final StatusSignal<?> hopperAIsDetected;   // boolean
    private final StatusSignal<?> hopperASignalStrength; // double (unitless)
    private final StatusSignal<?> hopperBDistance;
    private final StatusSignal<?> hopperBIsDetected;
    private final StatusSignal<?> hopperBSignalStrength;

    // ===== ToF detection thresholds (tune) =====
    // If you like mm in your head:
    private static final double HOPPER_A_THRESH_M = 0.25;   // 250mm
    private static final double HOPPER_A_HYST_M  = 0.02;    // 20mm
    private static final double HOPPER_B_THRESH_M = 0.25;
    private static final double HOPPER_B_HYST_M  = 0.02;

    // Helps ignore “garbage” weak returns (default is 2500 typical short-range) :contentReference[oaicite:3]{index=3}
    private static final double MIN_STRENGTH = 2500;

    // ===== Constants =====
    /**
     * Distance threshold in millimeters below which we consider a game piece "detected".
     * Tune this value based on your game piece size and sensor mounting.
     */
    
    /**
     * Creates a new IndexerIOHardware instance.
     * Configures motors and sensors to known good states.
     */
public IndexerIOHardware() {
    conveyorMotor = new TalonFXS(Constants.Indexer.CONVEYOR_MOTOR_ID, Constants.RIO_CANBUS);
    indexerMotor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID, Constants.RIO_CANBUS);

    conveyorMotor.getConfigurator().apply(TalonFXConfigs.conveyorConfig());
    indexerMotor.getConfigurator().apply(TalonFXConfigs.indexerConfig());

    // Get status signals
    conveyorVelocity = conveyorMotor.getVelocity();
    conveyorAppliedVolts = conveyorMotor.getMotorVoltage();
    conveyorCurrent = conveyorMotor.getSupplyCurrent();
    indexerVelocity = indexerMotor.getVelocity();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerCurrent = indexerMotor.getSupplyCurrent();

    hopperAToF = new CANrange(Constants.Indexer.HOPPER_A_TOF_ID, Constants.RIO_CANBUS);
    hopperBToF = new CANrange(Constants.Indexer.HOPPER_B_TOF_ID, Constants.RIO_CANBUS);

    // ---- Configure CANrange(s) ----
    var tofParams = new ToFParamsConfigs()
        .withUpdateMode(UpdateModeValue.ShortRange100Hz)  // update mode choice :contentReference[oaicite:6]{index=6}
        .withUpdateFrequency(50.0);                      // config range 5..50 Hz :contentReference[oaicite:7]{index=7}

    var proxA = new ProximityParamsConfigs()
        .withProximityThreshold(HOPPER_A_THRESH_M)       // meters :contentReference[oaicite:8]{index=8}
        .withProximityHysteresis(HOPPER_A_HYST_M)        // meters :contentReference[oaicite:9]{index=9}
        .withMinSignalStrengthForValidMeasurement(MIN_STRENGTH); // :contentReference[oaicite:10]{index=10}

    var proxB = new ProximityParamsConfigs()
        .withProximityThreshold(HOPPER_B_THRESH_M)
        .withProximityHysteresis(HOPPER_B_HYST_M)
        .withMinSignalStrengthForValidMeasurement(MIN_STRENGTH);

    CANrangeConfiguration aCfg = new CANrangeConfiguration();
    aCfg.ToFParams = tofParams;
    aCfg.ProximityParams = proxA;

    CANrangeConfiguration bCfg = new CANrangeConfiguration();
    bCfg.ToFParams = tofParams;
    bCfg.ProximityParams = proxB;

    hopperAToF.getConfigurator().apply(aCfg);
    hopperBToF.getConfigurator().apply(bCfg);

    // ---- Grab status signals ----
    hopperADistance = hopperAToF.getDistance();         // meters :contentReference[oaicite:11]{index=11}
    hopperAIsDetected = hopperAToF.getIsDetected();     // boolean :contentReference[oaicite:12]{index=12}
    hopperASignalStrength = hopperAToF.getSignalStrength(); // :contentReference[oaicite:13]{index=13}

    hopperBDistance = hopperBToF.getDistance();
    hopperBIsDetected = hopperBToF.getIsDetected();
    hopperBSignalStrength = hopperBToF.getSignalStrength();

    // Fast signals: 50Hz (every 20ms, matches robot loop)
//     BaseStatusSignal.setUpdateFrequencyForAll(
//         50.0

// );

    // Slow signals: 10Hz (diagnostics only, no need to refresh faster)
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        conveyorVelocity, conveyorAppliedVolts,
        indexerVelocity, indexerAppliedVolts,
        hopperASignalStrength, hopperBSignalStrength
        // conveyorCurrent, indexerCurrent,
        // hopperASignalStrength, hopperBSignalStrength
);

    conveyorMotor.optimizeBusUtilization();
    indexerMotor.optimizeBusUtilization();
    hopperAToF.optimizeBusUtilization();
    hopperBToF.optimizeBusUtilization();
}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Refresh all status signals efficiently

        BaseStatusSignal.refreshAll(
            conveyorVelocity, conveyorAppliedVolts,
            indexerVelocity, indexerAppliedVolts,
            hopperADistance, hopperAIsDetected,
            hopperBDistance, hopperBIsDetected
        );
        inputs.conveyorVelocityRPS = conveyorVelocity.getValueAsDouble();
        inputs.conveyorAppliedVolts = conveyorAppliedVolts.getValueAsDouble();
        inputs.indexerVelocityRPS = indexerVelocity.getValueAsDouble();
        inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();
        // inputs.hopperADistanceMm = hopperADistance.getValueAsDouble() * 1000.0; // m -> mm :contentReference[oaicite:14]{index=14}
        // inputs.hopperAHasPiece = (boolean) hopperAIsDetected.getValue();        // uses proximity config :contentReference[oaicite:15]{index=15}

        // inputs.hopperBDistanceMm = hopperBDistance.getValueAsDouble() * 1000.0;
        // inputs.hopperBHasPiece = (boolean) hopperBIsDetected.getValue();

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
} // End of class