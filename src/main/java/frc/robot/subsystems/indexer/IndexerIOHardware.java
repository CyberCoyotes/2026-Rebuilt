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
        /*API Breaking Changes [https://v6.docs.ctr-electronics.com/en/stable/docs/yearly-changes/yearly-changelog.html#breaking-changes]
        The new <Device>(int id, String canbus) constructors are now deprecated and will be removed in 2027.
        Use the new <Device>(int id, CANBus canbus) constructors instead.
        This change is intended to prepare users for 2027, where an explicit CAN bus declaration is necessary.
        */

        // Create motor objects
        // Note: Constants use "CONVEYOR" and "INDEXER" naming, we use "conveyor" and "indexer"
        conveyorMotor = new TalonFXS(Constants.Indexer.CONVEYOR_MOTOR_ID, Constants.RIO_CANBUS);
        indexerMotor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID, Constants.RIO_CANBUS);

        // Apply configurations from centralized config class
        conveyorMotor.getConfigurator().apply(TalonFXConfigs.conveyorConfig());
        indexerMotor.getConfigurator().apply(TalonFXConfigs.indexerConfig());

        // Hopper ToF sensors - detect game pieces at different positions
        /* These are CANrange time-of-flight sensors */
        // hopperAToF = new CANrange(Constants.Indexer.HOPPER_A_TOF_ID);
        // hopperAToF.setRangingMode(RangingMode.Short, 24);

        // hopperBToF = new CANrange(Constants.Indexer.HOPPER_B_TOF_ID);
        // hopperBToF.setRangingMode(RangingMode.Short, 24);

        // hopperCToF = new CANrange(Constants.Indexer.HOPPER_C_TOF_ID);
        // hopperCToF.setRangingMode(RangingMode.Short, 24);

        // Get status signals for efficient reading
        conveyorVelocity = conveyorMotor.getVelocity();
        conveyorAppliedVolts = conveyorMotor.getMotorVoltage();
        conveyorCurrent = conveyorMotor.getSupplyCurrent();


        indexerVelocity = indexerMotor.getVelocity();
        indexerAppliedVolts = indexerMotor.getMotorVoltage();
        indexerCurrent = indexerMotor.getSupplyCurrent();

        // Configure update frequencies for better performance
        BaseStatusSignal.setUpdateFrequencyForAll(
            0.0,  // 50Hz for current (important but not critical)
            conveyorVelocity, conveyorAppliedVolts,
            indexerVelocity, indexerAppliedVolts, conveyorCurrent, indexerCurrent);

        // Optimize bus utilization
        conveyorMotor.optimizeBusUtilization();
        indexerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Refresh all status signals efficiently
        // TODO: CAN signals stale on IDs 23/24 — commenting out refresh until
        // conveyor mechanical issue is resolved and wiring is verified
        
        // BaseStatusSignal.refreshAll(
        //     conveyorVelocity, conveyorAppliedVolts,
        //     indexerVelocity, indexerAppliedVolts
        // );
        // inputs.conveyorVelocityRPS = conveyorVelocity.getValueAsDouble();
        // inputs.conveyorAppliedVolts = conveyorAppliedVolts.getValueAsDouble();
        // inputs.indexerVelocityRPS = indexerVelocity.getValueAsDouble();
        // inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();

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