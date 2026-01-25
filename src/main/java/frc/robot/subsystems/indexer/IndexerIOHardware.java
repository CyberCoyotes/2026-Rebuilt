package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants;
import frc.robot.util.TalonFXConfigs;

/**
 * IndexerIOHardware - Real hardware implementation for the indexer subsystem.
 *
 * This class interfaces with:
 * - 2x Kraken X60 motors (floor + feeder)
 * - 1x Playing With Fusion Time-of-Flight sensor
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Optimized status signal updates for performance
 * - ToF sensor configured for short-range detection
 * - All telemetry logged via AdvantageKit
 */

@SuppressWarnings("unused") // Remove when we approach comp ready code

public class IndexerIOHardware implements IndexerIO {

    // ===== Hardware =====
    private final TalonFX floorMotor;
    private final TalonFX feederMotor;
    private final TimeOfFlight feederToF;

    // ===== Control Requests =====
    private final DutyCycleOut floorDutyCycleRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut feederDutyCycleRequest = new DutyCycleOut(0.0);

    // ===== Status Signals (for efficient reading) =====
    // Note: Phoenix 6 uses typed StatusSignals, we just read the value
    private final StatusSignal<?> floorVelocity;
    private final StatusSignal<?> floorAppliedVolts;
    private final StatusSignal<?> floorCurrent;
    private final StatusSignal<?> floorTemp;

    private final StatusSignal<?> feederVelocity;
    private final StatusSignal<?> feederAppliedVolts;
    private final StatusSignal<?> feederCurrent;
    private final StatusSignal<?> feederTemp;

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
        floorMotor = new TalonFX(Constants.Indexer.CONVEYOR_MOTOR_ID); // TODO add new CANbus arg 
        feederMotor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID); // TODO add new CANbus arg 

        // Apply configurations from centralized config class
        floorMotor.getConfigurator().apply(TalonFXConfigs.indexerConfig());
        feederMotor.getConfigurator().apply(TalonFXConfigs.indexerConfig());

        // Create and configure ToF sensor
        feederToF = new TimeOfFlight(Constants.Indexer.INDEXER_TOF_ID);
        feederToF.setRangingMode(RangingMode.Short, 24);  // Short range, 24ms sample time

        // Get status signals for efficient reading
        floorVelocity = floorMotor.getVelocity();
        floorAppliedVolts = floorMotor.getMotorVoltage();
        floorCurrent = floorMotor.getSupplyCurrent();
        floorTemp = floorMotor.getDeviceTemp();

        feederVelocity = feederMotor.getVelocity();
        feederAppliedVolts = feederMotor.getMotorVoltage();
        feederCurrent = feederMotor.getSupplyCurrent();
        feederTemp = feederMotor.getDeviceTemp();

        // Configure update frequencies for better performance
        // Critical signals: 100Hz, Less critical: 50Hz, Temperature: 4Hz
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,  // 100Hz for velocity and voltage (critical for control)
            floorVelocity, floorAppliedVolts,
            feederVelocity, feederAppliedVolts
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,  // 50Hz for current (important but not critical)
            floorCurrent, feederCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            4.0,  // 4Hz for temperature (slow-changing)
            floorTemp, feederTemp
        );

        // Optimize bus utilization
        floorMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Refresh all status signals efficiently
        BaseStatusSignal.refreshAll(
            floorVelocity, floorAppliedVolts, floorCurrent, floorTemp,
            feederVelocity, feederAppliedVolts, feederCurrent, feederTemp
        );

        // Update floor motor inputs
        inputs.floorVelocityRPS = floorVelocity.getValueAsDouble();
        inputs.floorAppliedVolts = floorAppliedVolts.getValueAsDouble();
        inputs.floorCurrentAmps = floorCurrent.getValueAsDouble();
        inputs.floorTempCelsius = floorTemp.getValueAsDouble();

        // Update feeder motor inputs
        inputs.feederVelocityRPS = feederVelocity.getValueAsDouble();
        inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
        inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
        inputs.feederTempCelsius = feederTemp.getValueAsDouble();

        // Update ToF sensor inputs
        inputs.tofDistanceMM = feederToF.getRange();
        inputs.tofValid = feederToF.isRangeValid();
        inputs.gamePieceDetected = inputs.tofValid &&
                                   inputs.tofDistanceMM < TOF_DETECTION_THRESHOLD_MM;
    }

    @Override
    public void setFloorMotor(double percent) {
        floorMotor.setControl(floorDutyCycleRequest.withOutput(percent));
    }

    @Override
    public void setFeederMotor(double percent) {
        feederMotor.setControl(feederDutyCycleRequest.withOutput(percent));
    }

    @Override
    public void stop() {
        floorMotor.stopMotor();
        feederMotor.stopMotor();
    }
}
