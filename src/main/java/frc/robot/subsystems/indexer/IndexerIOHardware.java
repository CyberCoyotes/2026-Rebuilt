package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit       = Constants.Indexer.ConveyorConfig.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = false; // TEMPORARILY DISABLED. RE-ENABLE WHEN LIMIT FOUND

        config.Voltage.PeakForwardVoltage = Constants.Indexer.ConveyorConfig.PEAK_FORWARD_VOLTAGE;
        config.Voltage.PeakReverseVoltage = Constants.Indexer.ConveyorConfig.PEAK_REVERSE_VOLTAGE;

        return config;
    }

    private static TalonFXConfiguration indexerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = Constants.Indexer.KickerConfig.NEUTRAL_MODE;
        config.MotorOutput.Inverted    = Constants.Indexer.KickerConfig.INVERTED;

        config.CurrentLimits.SupplyCurrentLimit       = Constants.Indexer.KickerConfig.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit       = Constants.Indexer.KickerConfig.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = Constants.Indexer.KickerConfig.PEAK_FORWARD_VOLTAGE;
        config.Voltage.PeakReverseVoltage = Constants.Indexer.KickerConfig.PEAK_REVERSE_VOLTAGE;

        
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Indexer.KickerConfig.OPEN_LOOP_RAMP_RATE;

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

    // == Hardware =================================================================
    private final TalonFX conveyorMotor;
    private final TalonFX kickerMotorLead;
    private final TalonFX kickerMotorFollow;
    private final CANrange chuteToF;

    // == Control Requests ==========================================================
    private final VoltageOut conveyorVoltageRequest = new VoltageOut(0.0);

    // Kicker follower was mechanically flipped again, so it must mirror the lead
    // motor with opposite shaft rotation while still matching the same commanded
    // game-piece motion.
    private final VoltageOut kickerLeadVoltageRequest  = new VoltageOut(0.0);
    private final Follower kickerFollowerRequest =
        new Follower(Constants.Indexer.KICKER_LEFT_MOTOR_ID, Constants.Indexer.KickerConfig.FOLLOWER_ALIGNMENT);

    // == Status Signals ===========================================================
    /* These Status Signals were not typed previously <?>, but trying Typed e.g. <AngularVelocity> */
    private final StatusSignal<AngularVelocity> conveyorVelocity;
    private final StatusSignal<Current> conveyorCurrent;
    private final StatusSignal<AngularVelocity> kickerLeadVelocity;
    private final StatusSignal<AngularVelocity> kickerFollowVelocity;
    private final StatusSignal<Current> kickerLeadCurrent;
    private final StatusSignal<Current> kickerFollowCurrent;
    private final StatusSignal<Distance> chuteDistance;
    private final StatusSignal<Boolean> chuteIsDetected;

    // == Constructor =============================================================
    public IndexerIOHardware() {
        conveyorMotor = new TalonFX(Constants.Indexer.CONVEYOR_MOTOR_ID, Constants.RIO_CANBUS);
        kickerMotorLead  = new TalonFX(Constants.Indexer.KICKER_LEFT_MOTOR_ID,   Constants.RIO_CANBUS);
        kickerMotorFollow = new TalonFX(Constants.Indexer.KICKER_RIGHT_MOTOR_ID,   Constants.RIO_CANBUS);
        chuteToF      = new CANrange(Constants.Indexer.CHUTE_TOF_ID,       Constants.RIO_CANBUS);

       // Apply configs with retry logic — replaces the single-attempt local helper.
        // Five retries handles devices still booting when apply() is first called.
        PhoenixUtil.applyConfig("Conveyor",  () -> conveyorMotor.getConfigurator().apply(conveyorConfig()));
        PhoenixUtil.applyConfig("Kicker Lead", () -> {
            StatusCode code = kickerMotorLead.getConfigurator().apply(indexerConfig());
            System.out.println("Kicker Lead config result: " + code.getName());
            return code;
        });                                                                         //TEMPORARY CHECK TO MAKE SURE MOTOR CONFIGS ARE BEING APPLIED CORRECTLY
        PhoenixUtil.applyConfig("Kicker Follow", () -> {
            StatusCode code = kickerMotorFollow.getConfigurator().apply(indexerConfig());
            System.out.println("Kicker Follow config result: " + code.getName());
            return code;
        });
        PhoenixUtil.applyConfig("Chute ToF", () -> chuteToF.getConfigurator().apply(chuteCANrangeConfig()));
        // Follower must be set after configs are applied.
        kickerMotorFollow.setControl(kickerFollowerRequest);

        conveyorVelocity = conveyorMotor.getVelocity();
        conveyorCurrent  = conveyorMotor.getSupplyCurrent();
        kickerLeadVelocity  = kickerMotorLead.getVelocity();
        kickerFollowVelocity = kickerMotorFollow.getVelocity();
        kickerLeadCurrent   = kickerMotorLead.getSupplyCurrent();
        kickerFollowCurrent = kickerMotorFollow.getSupplyCurrent();
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
            chuteDistance,          chuteIsDetected
        );

        inputs.conveyorVelocityRPS = conveyorVelocity.getValueAsDouble();
        inputs.conveyorCurrentAmps = conveyorCurrent.getValueAsDouble();
        inputs.kickerLeadVelocityRPS  = kickerLeadVelocity.getValueAsDouble();
        inputs.kickerLeadCurrentAmps  = kickerLeadCurrent.getValueAsDouble();
        inputs.kickerFollowCurrentAmps = kickerFollowCurrent.getValueAsDouble();
        inputs.chuteDistanceMeters = chuteDistance.getValueAsDouble();
        inputs.chuteDetected       = chuteIsDetected.getValue();
    }

    @Override
    public void setConveyorMotor(double volts) {
        conveyorMotor.setControl(conveyorVoltageRequest.withOutput(volts));
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
