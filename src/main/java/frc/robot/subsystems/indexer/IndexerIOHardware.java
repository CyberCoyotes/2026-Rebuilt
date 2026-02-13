package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants;
import frc.robot.util.TalonFXConfigs;

/**
 * IndexerIOHardware - Real hardware implementation for the indexer subsystem.
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Optimized status signal updates for performance
 * - ToF sensors configured for short-range detection
 * - All telemetry logged via AdvantageKit
 *
 * Motor naming convention:
 * - "conveyor" motor moves pieces along hopper conveyor
 * - "indexer" motor feeds pieces to shooter
 *
 * @see Constants.Indexer for hardware configuration
 * @see IndexerSubsystemBasic for a simpler direct-hardware approach (good for learning)
 */

@SuppressWarnings("unused") // Remove when we approach comp ready code

public class IndexerIOHardware implements IndexerIO {

    // ===== Hardware =====
    private final TalonFXS conveyorMotor;   // TalonFXS with Minion motor
    private final TalonFX indexerMotor;  // Also called "indexer" in Constants

    // Time-of-Flight sensors
    private final TimeOfFlight indexerToF;   // Detects game piece ready to shoot
    private final TimeOfFlight hopperAToF;  // Hopper position A
    private final TimeOfFlight hopperBToF;  // Hopper position B
    private final TimeOfFlight hopperCToF;  // Hopper position C

    // ===== Control Requests =====
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut indexerVoltageRequest = new VoltageOut(0.0);

    // ===== Status Signals (for efficient reading) =====
    // Note: Phoenix 6 uses typed StatusSignals, we just read the value
    private final StatusSignal<?> conveyorVelocity;
    private final StatusSignal<?> conveyorAppliedVolts;
    private final StatusSignal<?> conveyorCurrent;
    private final StatusSignal<?> conveyorTemp;

    private final StatusSignal<?> indexerVelocity;
    private final StatusSignal<?> indexerAppliedVolts;
    private final StatusSignal<?> indexerCurrent;
    private final StatusSignal<?> indexerTemp;

    // ===== Constants =====
    /**
     * Distance threshold in millimeters below which we consider a game piece "detected".
     * Tune this value based on your game piece size and sensor mounting.
     */
    private static final double TOF_DETECTION_THRESHOLD_MM = 100.0;  // TODO: Tune on robot

    /**
     * Creates a new IndexerIOHardware instance.
     * Configures motors and sensors to known good states.
     */
    public IndexerIOHardware() {
        /*API Breaking Changes [https://v6.docs.ctr-electronics.com/en/stable/docs/yearly-changes/yearly-changelog.html#breaking-changes]
        The new <Device>(int id, String canbus) constructors are now deprecated and will be removed in 2027.
        Use the new <Device>(int id, CANBus canbus) constructors instead.
        This change is intended to prepare users for 2027, where an explicit CAN bus declaration is necessary.
        */

        // Create motor objects
        // Note: Constants use "CONVEYOR" and "INDEXER" naming, we use "conveyor" and "indexer"
        conveyorMotor = new TalonFXS(Constants.Indexer.CONVEYOR_MOTOR_ID); // TODO add new CANbus arg
        indexerMotor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID); // TODO add new CANbus arg

        // Apply configurations from centralized config class
        conveyorMotor.getConfigurator().apply(TalonFXConfigs.conveyorConfig());
        indexerMotor.getConfigurator().apply(TalonFXConfigs.indexerConfig());

        // Create and configure ToF sensors
        // Indexer ToF - detects game piece ready to shoot
        indexerToF = new TimeOfFlight(Constants.Indexer.INDEXER_TOF_ID);
        indexerToF.setRangingMode(RangingMode.Short, 24);  // Short range, 24ms sample time

        // Hopper ToF sensors - detect game pieces at different positions
        hopperAToF = new TimeOfFlight(Constants.Indexer.HOPPER_TOP_A_TOF_ID);
        hopperAToF.setRangingMode(RangingMode.Short, 24);

        hopperBToF = new TimeOfFlight(Constants.Indexer.HOPPER_TOP_B_TOF_ID);
        hopperBToF.setRangingMode(RangingMode.Short, 24);

        hopperCToF = new TimeOfFlight(Constants.Indexer.HOPPER_TOP_C_TOF_ID);
        hopperCToF.setRangingMode(RangingMode.Short, 24);

        // Get status signals for efficient reading
        conveyorVelocity = conveyorMotor.getVelocity();
        conveyorAppliedVolts = conveyorMotor.getMotorVoltage();
        conveyorCurrent = conveyorMotor.getSupplyCurrent();
        conveyorTemp = conveyorMotor.getDeviceTemp();

        indexerVelocity = indexerMotor.getVelocity();
        indexerAppliedVolts = indexerMotor.getMotorVoltage();
        indexerCurrent = indexerMotor.getSupplyCurrent();
        indexerTemp = indexerMotor.getDeviceTemp();

        // Configure update frequencies for better performance
        // Critical signals: 100Hz, Less critical: 50Hz, Temperature: 4Hz
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,  // 100Hz for velocity and voltage (critical for control)
            conveyorVelocity, conveyorAppliedVolts,
            indexerVelocity, indexerAppliedVolts
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,  // 50Hz for current (important but not critical)
            conveyorCurrent, indexerCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            4.0,  // 4Hz for temperature (slow-changing)
            conveyorTemp, indexerTemp
        );

        // Optimize bus utilization
        conveyorMotor.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Refresh all status signals efficiently
        BaseStatusSignal.refreshAll(
            conveyorVelocity, conveyorAppliedVolts, conveyorCurrent, conveyorTemp,
            indexerVelocity, indexerAppliedVolts, indexerCurrent, indexerTemp
        );

        // Update conveyor motor inputs
        inputs.conveyorVelocityRPS = conveyorVelocity.getValueAsDouble();
        inputs.conveyorAppliedVolts = conveyorAppliedVolts.getValueAsDouble();
        inputs.conveyorCurrentAmps = conveyorCurrent.getValueAsDouble();
        inputs.conveyorTempCelsius = conveyorTemp.getValueAsDouble();

        // Update indexer motor inputs
        inputs.indexerVelocityRPS = indexerVelocity.getValueAsDouble();
        inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();
        inputs.indexerCurrentAmps = indexerCurrent.getValueAsDouble();
        inputs.indexerTempCelsius = indexerTemp.getValueAsDouble();

        // Update indexer ToF sensor inputs (detects game piece ready to shoot)
        inputs.tofDistanceMM = indexerToF.getRange();
        inputs.tofValid = indexerToF.isRangeValid();
        inputs.gamePieceDetected = inputs.tofValid &&
                                   inputs.tofDistanceMM < TOF_DETECTION_THRESHOLD_MM;

        // Update hopper A ToF sensor inputs
        inputs.hopperADistanceMM = hopperAToF.getRange();
        inputs.hopperAValid = hopperAToF.isRangeValid();
        inputs.hopperADetected = inputs.hopperAValid &&
                                 inputs.hopperADistanceMM < TOF_DETECTION_THRESHOLD_MM;

        // Update hopper B ToF sensor inputs
        inputs.hopperBDistanceMM = hopperBToF.getRange();
        inputs.hopperBValid = hopperBToF.isRangeValid();
        inputs.hopperBDetected = inputs.hopperBValid &&
                                 inputs.hopperBDistanceMM < TOF_DETECTION_THRESHOLD_MM;

        // Update hopper C ToF sensor inputs
        inputs.hopperCDistanceMM = hopperCToF.getRange();
        inputs.hopperCValid = hopperCToF.isRangeValid();
        inputs.hopperCDetected = inputs.hopperCValid &&
                                 inputs.hopperCDistanceMM < TOF_DETECTION_THRESHOLD_MM;
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
}
