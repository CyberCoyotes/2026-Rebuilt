package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;

/**
 * IndexerIOSim - Simulation implementation of IndexerIO.
 *
 * Instantiates the conveyor + kicker leader/follower TalonFX objects so they
 * appear in the Sim GUI, and drives leader rotors from DCMotorSim physics.
 * The chute CANrange is not instantiated; chuteDetected is exposed as a
 * software flag via simulateGamePieceInChute() for test injection.
 */
public class IndexerIOSim implements IndexerIO {

    private static final double LOOP_PERIOD_SEC = 0.020;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getKrakenX60(1);

    private final TalonFX conveyor;
    private final TalonFX kickerLeader;
    private final TalonFX kickerFollower;

    private final TalonFXSimState conveyorSim;
    private final TalonFXSimState kickerLeaderSim;

    private final DCMotorSim conveyorPlant;
    private final DCMotorSim kickerPlant;

    private final VelocityVoltage conveyorVelocityRequest =
        new VelocityVoltage(0.0).withSlot(0).withEnableFOC(false);
    private final VoltageOut conveyorVoltageRequest =
        new VoltageOut(0.0).withEnableFOC(false);
    private final VelocityVoltage kickerVelocityRequest =
        new VelocityVoltage(0.0).withSlot(0).withEnableFOC(false);
    private final VoltageOut kickerVoltageRequest =
        new VoltageOut(0.0).withEnableFOC(false);

    private boolean chuteDetected = false;

    public IndexerIOSim() {
        conveyor       = new TalonFX(Constants.Indexer.CONVEYOR_MOTOR_ID, Constants.RIO_CANBUS);
        kickerLeader   = new TalonFX(Constants.Indexer.KICKER_LEFT_MOTOR_ID, Constants.RIO_CANBUS);
        kickerFollower = new TalonFX(Constants.Indexer.KICKER_RIGHT_MOTOR_ID, Constants.RIO_CANBUS);

        conveyor.getConfigurator().apply(conveyorConfig());
        kickerLeader.getConfigurator().apply(kickerLeaderConfig());
        kickerFollower.getConfigurator().apply(kickerFollowerConfig());

        kickerFollower.setControl(new Follower(
            Constants.Indexer.KICKER_LEFT_MOTOR_ID,
            Constants.Indexer.KickerFollowerConfig.FOLLOWER_ALIGNMENT));

        conveyorSim = conveyor.getSimState();
        kickerLeaderSim = kickerLeader.getSimState();

        conveyorPlant = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(MOTOR_GEARBOX, 0.001, 1.0),
            MOTOR_GEARBOX);
        kickerPlant = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(MOTOR_GEARBOX, 0.001, 1.0),
            MOTOR_GEARBOX);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        stepPhysics(conveyorSim, conveyorPlant);
        stepPhysics(kickerLeaderSim, kickerPlant);

        inputs.conveyorVelocityRPS = conveyor.getVelocity().getValueAsDouble();
        inputs.conveyorCurrentAmps = conveyor.getSupplyCurrent().getValueAsDouble();
        inputs.kickerLeadVelocityRPS = kickerLeader.getVelocity().getValueAsDouble();
        inputs.kickerLeadCurrentAmps = kickerLeader.getSupplyCurrent().getValueAsDouble();
        inputs.kickerFollowCurrentAmps = kickerFollower.getSupplyCurrent().getValueAsDouble();

        inputs.chuteDistanceMeters = chuteDetected ? 0.05 : 1.0;
        inputs.chuteDetected = chuteDetected;
    }

    @Override
    public void setConveyorVelocity(double rps) {
        conveyor.setControl(conveyorVelocityRequest.withVelocity(rps));
    }

    @Override
    public void setConveyorMotor(double volts) {
        conveyor.setControl(conveyorVoltageRequest.withOutput(volts));
    }

    @Override
    public void setKickerVelocity(double rps) {
        kickerLeader.setControl(kickerVelocityRequest.withVelocity(rps));
    }

    @Override
    public void setKickerMotorVolts(double volts) {
        kickerLeader.setControl(kickerVoltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        conveyor.stopMotor();
        kickerLeader.stopMotor();
    }

    /** Call from test code or SmartDashboard to inject a simulated game piece. */
    public void simulateGamePieceInChute(boolean present) {
        chuteDetected = present;
    }

    // == Helpers ==============================================================
    private static void stepPhysics(TalonFXSimState simState, DCMotorSim plant) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        plant.setInputVoltage(simState.getMotorVoltage());
        plant.update(LOOP_PERIOD_SEC);
        simState.setRawRotorPosition(plant.getAngularPositionRotations());
        simState.setRotorVelocity(plant.getAngularVelocityRPM() / 60.0);
    }

    private static TalonFXConfiguration conveyorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.Indexer.ConveyorConfig.NEUTRAL_MODE;
        config.MotorOutput.Inverted = Constants.Indexer.ConveyorConfig.INVERTED;
        config.Slot0.kV = Constants.Indexer.ConveyorConfig.SLOT0_KV;
        config.Slot0.kP = Constants.Indexer.ConveyorConfig.SLOT0_KP;
        return config;
    }

    private static TalonFXConfiguration kickerLeaderConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.Indexer.KickerLeaderConfig.NEUTRAL_MODE;
        config.MotorOutput.Inverted = Constants.Indexer.KickerLeaderConfig.INVERTED;
        config.Slot0.kV = Constants.Indexer.KickerLeaderConfig.SLOT0_KV;
        config.Slot0.kP = Constants.Indexer.KickerLeaderConfig.SLOT0_KP;
        return config;
    }

    private static TalonFXConfiguration kickerFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.Indexer.KickerFollowerConfig.NEUTRAL_MODE;
        return config;
    }
}
