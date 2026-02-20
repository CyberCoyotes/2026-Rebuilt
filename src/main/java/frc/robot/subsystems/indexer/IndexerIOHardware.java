package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants;
import frc.robot.util.TalonFXConfigs;

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
    // private final CANrange hopperAToF; 
    // private final CANrange hopperBToF;
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

    // Fast signals: 50Hz (every 20ms, matches robot loop)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        conveyorVelocity, conveyorAppliedVolts,
        indexerVelocity, indexerAppliedVolts);

    // Slow signals: 10Hz (diagnostics only, no need to refresh faster)
    BaseStatusSignal.setUpdateFrequencyForAll(
        10.0,
        conveyorCurrent, indexerCurrent);

    conveyorMotor.optimizeBusUtilization();
    indexerMotor.optimizeBusUtilization();
}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Refresh all status signals efficiently

        BaseStatusSignal.refreshAll(
            conveyorVelocity, conveyorAppliedVolts,
            indexerVelocity, indexerAppliedVolts
        );
        inputs.conveyorVelocityRPS = conveyorVelocity.getValueAsDouble();
        inputs.conveyorAppliedVolts = conveyorAppliedVolts.getValueAsDouble();
        inputs.indexerVelocityRPS = indexerVelocity.getValueAsDouble();
        inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();

    }

    @Override
    public void setConveyorMotor(double percent) {
        conveyorMotor.setControl(conveyorVoltageRequest .withOutput(percent));
    }

    @Override
    public void setIndexerMotor(double percent) {
        indexerMotor.setControl(indexerVoltageRequest.withOutput(percent));
    }

    @Override
    public void stop() {
        conveyorMotor.stopMotor();
        indexerMotor.stopMotor();
    }
} // End of class