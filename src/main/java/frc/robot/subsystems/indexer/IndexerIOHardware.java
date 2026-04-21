package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.utilities.PhoenixUtil;

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

        config.MotorOutput.NeutralMode = Constants.Indexer.ConveyorConfig.NEUTRAL_MODE;
        config.MotorOutput.Inverted    = Constants.Indexer.ConveyorConfig.INVERTED;

        config.CurrentLimits.SupplyCurrentLimit       = Constants.Indexer.ConveyorConfig.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = Constants.Indexer.ConveyorConfig.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = Constants.Indexer.ConveyorConfig.PEAK_FORWARD_VOLTAGE;
        config.Voltage.PeakReverseVoltage = Constants.Indexer.ConveyorConfig.PEAK_REVERSE_VOLTAGE;

        // Slot 0 — used by VelocityVoltage for conveyor forward
        config.Slot0.kV = Constants.Indexer.ConveyorConfig.SLOT0_KV;
        config.Slot0.kP = Constants.Indexer.ConveyorConfig.SLOT0_KP;

        return config;
    }

    private static TalonFXConfiguration kickerLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = Constants.Indexer.KickerLeaderConfig.NEUTRAL_MODE;
        config.MotorOutput.Inverted    = Constants.Indexer.KickerLeaderConfig.INVERTED;

        config.CurrentLimits.SupplyCurrentLimit       = Constants.Indexer.KickerLeaderConfig.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = Constants.Indexer.KickerLeaderConfig.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = Constants.Indexer.KickerLeaderConfig.PEAK_FORWARD_VOLTAGE;
        config.Voltage.PeakReverseVoltage = Constants.Indexer.KickerLeaderConfig.PEAK_REVERSE_VOLTAGE;

        // Slot 0 — used by VelocityVoltage for kicker forward
        config.Slot0.kV = Constants.Indexer.KickerLeaderConfig.SLOT0_KV;
        config.Slot0.kP = Constants.Indexer.KickerLeaderConfig.SLOT0_KP;

        return config;
    }

    // While in follower mode, direction is governed by the Follower request — INVERTED not set.
    private static TalonFXConfiguration kickerFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = Constants.Indexer.KickerFollowerConfig.NEUTRAL_MODE;

        config.CurrentLimits.SupplyCurrentLimit       = Constants.Indexer.KickerFollowerConfig.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = Constants.Indexer.KickerFollowerConfig.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = Constants.Indexer.KickerFollowerConfig.PEAK_FORWARD_VOLTAGE;
        config.Voltage.PeakReverseVoltage = Constants.Indexer.KickerFollowerConfig.PEAK_REVERSE_VOLTAGE;

        return config;
    }

    private static CANrangeConfiguration chuteCANrangeConfig() {
        CANrangeConfiguration config = new CANrangeConfiguration();

        config.ProximityParams.ProximityThreshold = Constants.Indexer.FUEL_DETECTION_DISTANCE;
        config.ProximityParams.ProximityHysteresis = Constants.Indexer.ChuteSensorConfig.PROXIMITY_HYSTERESIS;
        config.FovParams.FOVRangeX = Constants.Indexer.ChuteSensorConfig.FOV_RANGE_X;
        config.FovParams.FOVRangeY = Constants.Indexer.ChuteSensorConfig.FOV_RANGE_Y;

        return config;
    }

    private static CANrangeConfiguration hopperTopCANrangeConfig() {
        CANrangeConfiguration config = new CANrangeConfiguration();

        config.ProximityParams.ProximityThreshold = Constants.Indexer.FUEL_DETECTION_DISTANCE;
        config.ProximityParams.ProximityHysteresis = Constants.Indexer.ChuteSensorConfig.PROXIMITY_HYSTERESIS;
        config.FovParams.FOVRangeX = Constants.Indexer.ChuteSensorConfig.FOV_RANGE_X;
        config.FovParams.FOVRangeY = Constants.Indexer.ChuteSensorConfig.FOV_RANGE_Y;

        return config;
    }

    // == Hardware =================================================================
    private final TalonFX conveyorMotor;
    private final TalonFX kickerMotorLead;
    private final TalonFX kickerMotorFollow;
    private final CANrange hopperTopToF;
    private final CANrange chuteToF;

    // == Control Requests ==========================================================
    // Conveyor forward uses VelocityVoltage (Slot 0). Reverse/popper still use VoltageOut.
    // Fallback: private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0).withEnableFOC(false);
    private final VelocityVoltage conveyorVelocityRequest = new VelocityVoltage(0.0).withSlot(0).withEnableFOC(false);
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0).withEnableFOC(false);
    // Kicker forward uses VelocityVoltage (Slot 0). Reverse/popper still use VoltageOut.
    // Fallback: private final VoltageOut kickerLeadVoltageRequest = new VoltageOut(0.0).withEnableFOC(false);
    private final VelocityVoltage kickerLeadVelocityRequest = new VelocityVoltage(0.0).withSlot(0).withEnableFOC(false);
    private final VoltageOut kickerLeadVoltageRequest  = new VoltageOut(0.0).withEnableFOC(false);
    private final Follower kickerFollowerRequest =
        new Follower(Constants.Indexer.KICKER_LEFT_MOTOR_ID, Constants.Indexer.KickerFollowerConfig.FOLLOWER_ALIGNMENT);

    // == Status Signals ===========================================================
    /* These Status Signals were not typed previously <?>, but trying Typed e.g. <AngularVelocity> */
    private final StatusSignal<AngularVelocity> conveyorVelocity;
    private final StatusSignal<Current> conveyorCurrent;
    private final StatusSignal<AngularVelocity> kickerLeadVelocity;
    private final StatusSignal<AngularVelocity> kickerFollowVelocity;
    private final StatusSignal<Current> kickerLeadCurrent;
    private final StatusSignal<Current> kickerFollowCurrent;
    private final StatusSignal<Boolean> hopperTopIsDetected;
    private final StatusSignal<Distance> chuteDistance;
    private final StatusSignal<Boolean> chuteIsDetected;

    // == Constructor =============================================================
    public IndexerIOHardware() {
        conveyorMotor = new TalonFX(Constants.Indexer.CONVEYOR_MOTOR_ID, Constants.RIO_CANBUS);
        kickerMotorLead  = new TalonFX(Constants.Indexer.KICKER_LEFT_MOTOR_ID,   Constants.RIO_CANBUS);
        kickerMotorFollow = new TalonFX(Constants.Indexer.KICKER_RIGHT_MOTOR_ID,   Constants.RIO_CANBUS);
        hopperTopToF  = new CANrange(Constants.Indexer.HOPPER_TOF_ID,       Constants.RIO_CANBUS);
        chuteToF      = new CANrange(Constants.Indexer.CHUTE_TOF_ID,       Constants.RIO_CANBUS);

       // Apply configs with retry logic — replaces the single-attempt local helper.
        // Five retries handles devices still booting when apply() is first called.
        PhoenixUtil.applyConfig("Conveyor",  () -> conveyorMotor.getConfigurator().apply(conveyorConfig()));
        PhoenixUtil.applyConfig("Kicker Lead", () -> {
            StatusCode code = kickerMotorLead.getConfigurator().apply(kickerLeaderConfig());
            System.out.println("Kicker Lead config result: " + code.getName());
            return code;
        });                                                                         //TEMPORARY CHECK TO MAKE SURE MOTOR CONFIGS ARE BEING APPLIED CORRECTLY
        PhoenixUtil.applyConfig("Kicker Follow", () -> {
            StatusCode code = kickerMotorFollow.getConfigurator().apply(kickerFollowerConfig());
            System.out.println("Kicker Follow config result: " + code.getName());
            return code;
        });
        PhoenixUtil.applyConfig("Hopper Top ToF", () -> hopperTopToF.getConfigurator().apply(hopperTopCANrangeConfig()));
        PhoenixUtil.applyConfig("Chute ToF", () -> chuteToF.getConfigurator().apply(chuteCANrangeConfig()));
        // Follower must be set after configs are applied.
        kickerMotorFollow.setControl(kickerFollowerRequest);

        conveyorVelocity = conveyorMotor.getVelocity();
        conveyorCurrent  = conveyorMotor.getSupplyCurrent();
        kickerLeadVelocity  = kickerMotorLead.getVelocity();
        kickerFollowVelocity = kickerMotorFollow.getVelocity();
        kickerLeadCurrent   = kickerMotorLead.getSupplyCurrent();
        kickerFollowCurrent = kickerMotorFollow.getSupplyCurrent();
        hopperTopIsDetected = hopperTopToF.getIsDetected();
        chuteDistance    = chuteToF.getDistance();
        chuteIsDetected  = chuteToF.getIsDetected();
    }

    // == IO Implementation ========================================================
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            conveyorVelocity,       conveyorCurrent,
            kickerLeadVelocity,     kickerLeadCurrent,
            kickerFollowCurrent,    kickerFollowVelocity,
            hopperTopIsDetected,
            chuteDistance,          chuteIsDetected
        );

        inputs.conveyorVelocityRPS = conveyorVelocity.getValueAsDouble();
        inputs.conveyorCurrentAmps = conveyorCurrent.getValueAsDouble();
        inputs.kickerLeadVelocityRPS  = kickerLeadVelocity.getValueAsDouble();
        inputs.kickerLeadCurrentAmps  = kickerLeadCurrent.getValueAsDouble();
        inputs.kickerFollowCurrentAmps = kickerFollowCurrent.getValueAsDouble();
        inputs.hopperTopDetected = hopperTopIsDetected.getValue();
        inputs.chuteDistanceMeters = chuteDistance.getValueAsDouble();
        inputs.chuteDetected       = chuteIsDetected.getValue();
    }

    @Override
    public void setConveyorVelocity(double rps) {
        conveyorMotor.setControl(conveyorVelocityRequest.withVelocity(rps));
    }

    @Override
    public void setConveyorMotor(double volts) {
        conveyorMotor.setControl(conveyorVoltageRequest.withOutput(volts));
    }

    @Override
    public void setKickerVelocity(double rps) {
        kickerMotorLead.setControl(kickerLeadVelocityRequest.withVelocity(rps));
    }

    @Override
    public void setKickerMotorVolts(double volts) {
        kickerMotorLead.setControl(kickerLeadVoltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        conveyorMotor.stopMotor();
        // Stop lead only — follower mirrors the leader's NeutralOut automatically.
        kickerMotorLead.stopMotor();
    }
}
