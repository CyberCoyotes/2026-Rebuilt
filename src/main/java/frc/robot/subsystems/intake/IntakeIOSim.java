package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;

/**
 * IntakeIOSim - Simulation implementation of IntakeIO.
 *
 * Instantiates real CTRE TalonFX objects (so they appear in the WPILib Sim GUI)
 * and drives their SimState from DCMotorSim physics models. The TalonFX runs its
 * own closed-loop on the simulated rotor data, matching the production behavior.
 */
public class IntakeIOSim implements IntakeIO {

    private static final double LOOP_PERIOD_SEC = 0.020;
    private static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60(1);
    private static final DCMotor SLIDE_GEARBOX = DCMotor.getKrakenX60(1);

    private final TalonFX roller;
    private final TalonFX slide;

    private final TalonFXSimState rollerSim;
    private final TalonFXSimState slideSim;

    private final DCMotorSim rollerPlant;
    private final DCMotorSim slidePlant;

    private final VelocityVoltage rollerRequest = new VelocityVoltage(0.0).withSlot(0);
    private final MotionMagicVoltage slideRequest = new MotionMagicVoltage(0.0).withSlot(0);

    public IntakeIOSim() {
        roller = new TalonFX(Constants.Intake.ROLLER_LEFT_MOTOR_ID, Constants.RIO_CANBUS);
        slide  = new TalonFX(Constants.Intake.SLIDE_MOTOR_ID, Constants.RIO_CANBUS);

        roller.getConfigurator().apply(rollerConfig());
        slide.getConfigurator().apply(slideConfig());
        slide.setPosition(0.0);

        rollerSim = roller.getSimState();
        slideSim  = slide.getSimState();

        rollerPlant = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, 0.001, 1.0),
            ROLLER_GEARBOX);
        slidePlant = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(SLIDE_GEARBOX, 0.025, 1.0),
            SLIDE_GEARBOX);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        stepPhysics(rollerSim, rollerPlant);
        stepPhysics(slideSim, slidePlant);

        inputs.slidePositionRotations = slide.getPosition().getValueAsDouble();
        inputs.slideVelocityRPS = slide.getVelocity().getValueAsDouble();
    }

    @Override
    public void setRollerVelocity(double rps) {
        roller.setControl(rollerRequest.withVelocity(rps));
    }

    @Override
    public void stopRoller() {
        roller.stopMotor();
    }

    @Override
    public void setSlidePosition(double position) {
        slide.setControl(slideRequest.withPosition(position));
    }

    @Override
    public void setSlidePositionSlow(double position) {
        slide.setControl(slideRequest.withPosition(position));
    }

    @Override
    public void stopSlide() {
        slide.stopMotor();
    }

    @Override
    public void resetSlideEncoder() {
        slide.setPosition(0.0);
        slidePlant.setState(0.0, 0.0);
    }

    // == Helpers ==============================================================
    private static void stepPhysics(TalonFXSimState simState, DCMotorSim plant) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        plant.setInputVoltage(simState.getMotorVoltage());
        plant.update(LOOP_PERIOD_SEC);
        simState.setRawRotorPosition(plant.getAngularPositionRotations());
        simState.setRotorVelocity(plant.getAngularVelocityRPM() / 60.0);
    }

    private static TalonFXConfiguration rollerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.Intake.RollerLeaderConfig.NEUTRAL_MODE;
        config.Slot0.kS = Constants.Intake.RollerLeaderConfig.KS;
        config.Slot0.kV = Constants.Intake.RollerLeaderConfig.KV;
        config.Slot0.kP = Constants.Intake.RollerLeaderConfig.KP;
        return config;
    }

    private static TalonFXConfiguration slideConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = Constants.Intake.SlideConfig.NEUTRAL_MODE;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.SLIDE_MAX_POS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.SlideConfig.REVERSE_SOFT_LIMIT;

        config.Slot0.kP = Constants.Intake.SlideConfig.KP;
        config.Slot0.kI = Constants.Intake.SlideConfig.KI;
        config.Slot0.kD = Constants.Intake.SlideConfig.KD;
        config.Slot0.kS = Constants.Intake.SlideConfig.KS;
        config.Slot0.kV = Constants.Intake.SlideConfig.KV;
        config.Slot0.kA = Constants.Intake.SlideConfig.KA;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.SLIDE_MM_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = Constants.Intake.SLIDE_MM_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = Constants.Intake.SLIDE_MM_JERK;

        return config;
    }
}
