package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import frc.robot.Constants;

/**
 * ShooterIOSim - Simulation implementation of ShooterIO.
 *
 * Instantiates real CTRE flywheel + hood hardware so they appear in the Sim GUI,
 * then drives their SimState from FlywheelSim (flywheel) and DCMotorSim (hood)
 * physics models. The follower flywheel is wired but its rotor state is not driven.
 */
public class ShooterIOSim implements ShooterIO {

    private static final double LOOP_PERIOD_SEC = 0.020;
    private static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60(2);
    // Hood is a Minion via TalonFXS; getKrakenX60 is used as a stand-in DC model for sim.
    private static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60(1);

    private final TalonFX flywheelLeader;
    private final TalonFX flywheelFollower;
    private final TalonFXS hood;

    private final TalonFXSimState flywheelLeaderSim;
    private final TalonFXSSimState hoodSim;

    private final FlywheelSim flywheelPlant;
    private final DCMotorSim hoodPlant;

    private final MotionMagicVelocityVoltage flywheelRequest =
        new MotionMagicVelocityVoltage(0.0).withEnableFOC(false);
    private final PositionVoltage hoodPositionRequest =
        new PositionVoltage(0.0).withEnableFOC(false);
    private final VoltageOut hoodVoltageRequest =
        new VoltageOut(0.0).withEnableFOC(false);

    public ShooterIOSim() {
        flywheelLeader   = new TalonFX(Constants.Flywheel.FLYWHEEL_LEFT_MOTOR_ID, Constants.RIO_CANBUS);
        flywheelFollower = new TalonFX(Constants.Flywheel.FLYWHEEL_RIGHT_MOTOR_ID, Constants.RIO_CANBUS);
        hood = new TalonFXS(Constants.Hood.HOOD_MOTOR_ID, Constants.RIO_CANBUS);

        flywheelLeader.getConfigurator().apply(flywheelLeaderConfig());
        flywheelFollower.getConfigurator().apply(flywheelFollowerConfig());
        hood.getConfigurator().apply(hoodConfig());

        // Wire follower so it appears in the GUI mirroring the leader's command
        flywheelFollower.setControl(new Follower(
            Constants.Flywheel.FLYWHEEL_LEFT_MOTOR_ID,
            Constants.Flywheel.FollowerConfig.FOLLOWER_ALIGNMENT));

        hood.setPosition(Constants.Hood.ENCODER_ZERO_POSITION);

        flywheelLeaderSim = flywheelLeader.getSimState();
        hoodSim = hood.getSimState();

        flywheelPlant = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(FLYWHEEL_GEARBOX, Constants.Flywheel.FLYWHEEL_MOI_KG_M2, 1.0),
            FLYWHEEL_GEARBOX);
        hoodPlant = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(HOOD_GEARBOX, 0.004, 1.0),
            HOOD_GEARBOX);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Flywheel physics: FlywheelSim is velocity-only — no rotor position to set.
        flywheelLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        flywheelPlant.setInputVoltage(flywheelLeaderSim.getMotorVoltage());
        flywheelPlant.update(LOOP_PERIOD_SEC);
        flywheelLeaderSim.setRotorVelocity(flywheelPlant.getAngularVelocityRPM() / 60.0);

        // Hood physics
        hoodSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        hoodPlant.setInputVoltage(hoodSim.getMotorVoltage());
        hoodPlant.update(LOOP_PERIOD_SEC);
        hoodSim.setRawRotorPosition(hoodPlant.getAngularPositionRotations());
        hoodSim.setRotorVelocity(hoodPlant.getAngularVelocityRPM() / 60.0);

        double motorRPS = flywheelLeader.getVelocity().getValueAsDouble();
        inputs.flywheelLeaderMotorRPS = motorRPS;
        inputs.flywheelLeaderMotorRPM = motorRPS * 60.0;
        inputs.flywheelAppliedVolts = flywheelLeader.getMotorVoltage().getValueAsDouble();
        inputs.hoodPositionRotations = hood.getPosition().getValueAsDouble();
    }

    @Override
    public void updateSlowInputs(ShooterIOInputs inputs) {
        inputs.flywheelCurrentAmps = flywheelLeader.getSupplyCurrent().getValueAsDouble();
        inputs.flywheelMaxTempCelsius = 25.0;
        inputs.hoodAppliedVolts = hood.getMotorVoltage().getValueAsDouble();
        inputs.hoodCurrentAmps = hood.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        flywheelLeader.setControl(flywheelRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public void setFlywheelVelocityTorqueFOC(double rpm) {
        flywheelLeader.setControl(flywheelRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public void stopFlywheels() {
        flywheelLeader.stopMotor();
    }

    @Override
    public void setHoodPose(double rawPosition) {
        hood.setControl(hoodPositionRequest.withPosition(rawPosition));
    }

    @Override
    public void setHoodVoltage(double volts) {
        hood.setControl(hoodVoltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        flywheelLeader.stopMotor();
        hood.stopMotor();
    }

    // == Configs (subset — closed-loop + soft limits) ========================
    private static TalonFXConfiguration flywheelLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.Flywheel.LeaderConfig.NEUTRAL_MODE;
        config.MotorOutput.Inverted = Constants.Flywheel.LeaderConfig.INVERTED;

        config.MotionMagic.MotionMagicAcceleration = Constants.Flywheel.MM_ACCELERATION_RPS_PER_SEC;
        config.MotionMagic.MotionMagicJerk = Constants.Flywheel.MM_JERK_RPS_PER_SEC_CUBED;

        config.Slot0.kP = Constants.Flywheel.KP;
        config.Slot0.kV = Constants.Flywheel.KV;
        config.Slot0.kA = Constants.Flywheel.KA;
        config.Slot0.kD = Constants.Flywheel.KD;
        return config;
    }

    private static TalonFXConfiguration flywheelFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.Flywheel.FollowerConfig.NEUTRAL_MODE;
        return config;
    }

    private static TalonFXSConfiguration hoodConfig() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = Constants.Hood.HoodConfig.MOTOR_ARRANGEMENT;
        config.MotorOutput.NeutralMode = Constants.Hood.HoodConfig.NEUTRAL_MODE;
        config.MotorOutput.Inverted = Constants.Hood.HoodConfig.INVERTED;

        config.Voltage.PeakForwardVoltage = Constants.Hood.PEAK_FORWARD_VOLTAGE;
        config.Voltage.PeakReverseVoltage = Constants.Hood.PEAK_REVERSE_VOLTAGE;

        config.Slot0.kP = Constants.Hood.KP;
        config.Slot0.kI = Constants.Hood.KI;
        config.Slot0.kD = Constants.Hood.KD;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Hood.MAX_POSE;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Hood.MIN_POSE;
        return config;
    }
}
