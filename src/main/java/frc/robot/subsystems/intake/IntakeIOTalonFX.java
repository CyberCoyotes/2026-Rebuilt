package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.util.TalonFXConfigs;

/**
 * IntakeIOTalonFX - Real hardware implementation using CTRE TalonFX motors.
 *
 * This class interfaces with:
 * - 1x TalonFX (rotator) for spinning intake wheels
 * - 1x TalonFX (slide) for extending/retracting intake
 *
 * Key features:
 * - Uses centralized TalonFXConfigs for motor configuration
 * - Percent output control for rotator (simple and responsive)
 * - Position control for slide (precise positioning)
 * - Optimized status signal updates for performance
 * - All telemetry logged via AdvantageKit
 *
 * @author @Isaak3
 */
public class IntakeIOTalonFX implements IntakeIO {

    // ===== Hardware =====
    private final TalonFX rotatorMotor;
    private final TalonFX slideMotor;
    private final CANBus kCANBus = new CANBus("rio");


    // ===== Control Requests =====
    private final DutyCycleOut rotatorDutyCycleRequest = new DutyCycleOut(0.0);
    private final PositionVoltage slidePositionRequest = new PositionVoltage(0.0);

    // ===== Status Signals (for efficient reading) =====
    // Rotator
    private final StatusSignal<?> rotatorVelocity;
    private final StatusSignal<?> rotatorVoltage;
    private final StatusSignal<?> rotatorCurrent;
    private final StatusSignal<?> rotatorTemp;

    // Slide
    private final StatusSignal<?> slidePosition;
    private final StatusSignal<?> slideVelocity;
    private final StatusSignal<?> slideVoltage;
    private final StatusSignal<?> slideCurrent;
    private final StatusSignal<?> slideTemp;

    // ===== Conversion Constants =====
    /** Gear ratio from slide motor to slide mechanism (motor rotations per slide extension) */
    private static final double SLIDE_GEAR_RATIO = 10.0;  // TODO: Measure actual ratio

    /**
     * Creates a new IntakeIOTalonFX instance.
     * Configures motors to known good states.
     */
    public IntakeIOTalonFX() {
        // Create motor objects

        rotatorMotor = new TalonFX(Constants.Intake.INTAKE_ROTATOR_ID, kCANBus);
        slideMotor = new TalonFX(Constants.Intake.INTAKE_SLIDE_ID, kCANBus);

        // Apply configurations from centralized config class
        rotatorMotor.getConfigurator().apply(TalonFXConfigs.intakeConfig());
        slideMotor.getConfigurator().apply(TalonFXConfigs.intakeConfig());

        // Get status signals for efficient reading
        // Rotator
        rotatorVelocity = rotatorMotor.getVelocity();
        rotatorVoltage = rotatorMotor.getMotorVoltage();
        rotatorCurrent = rotatorMotor.getSupplyCurrent();
        rotatorTemp = rotatorMotor.getDeviceTemp();

        // Slide
        slidePosition = slideMotor.getPosition();
        slideVelocity = slideMotor.getVelocity();
        slideVoltage = slideMotor.getMotorVoltage();
        slideCurrent = slideMotor.getSupplyCurrent();
        slideTemp = slideMotor.getDeviceTemp();

        // Configure update frequencies for better performance
        // Critical signals: 100Hz, Less critical: 50Hz, Temperature: 4Hz
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,  // 100Hz for velocity/position/voltage (critical for control)
            rotatorVelocity, rotatorVoltage,
            slidePosition, slideVelocity, slideVoltage
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,  // 50Hz for current
            rotatorCurrent, slideCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            4.0,  // 4Hz for temperature
            rotatorTemp, slideTemp
        );

        // Optimize bus utilization
        rotatorMotor.optimizeBusUtilization();
        slideMotor.optimizeBusUtilization();

        // Zero slide encoder at startup (assumes slide is retracted at startup)
        slideMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        // Refresh all status signals efficiently
        BaseStatusSignal.refreshAll(
            rotatorVelocity, rotatorVoltage, rotatorCurrent, rotatorTemp,
            slidePosition, slideVelocity, slideVoltage, slideCurrent, slideTemp
        );

        // Rotator motor data
        inputs.rotatorVelocityRPS = rotatorVelocity.getValueAsDouble();
        inputs.rotatorAppliedVolts = rotatorVoltage.getValueAsDouble();
        inputs.rotatorCurrentAmps = rotatorCurrent.getValueAsDouble();
        inputs.rotatorTempCelsius = rotatorTemp.getValueAsDouble();

        // Slide motor data
        inputs.slidePositionRotations = slidePosition.getValueAsDouble();
        inputs.slideVelocityRPS = slideVelocity.getValueAsDouble();
        inputs.slideAppliedVolts = slideVoltage.getValueAsDouble();
        inputs.slideCurrentAmps = slideCurrent.getValueAsDouble();
        inputs.slideTempCelsius = slideTemp.getValueAsDouble();
    }

    @Override
    public void setRotator(double percent) {
        rotatorMotor.setControl(rotatorDutyCycleRequest.withOutput(percent));
    }

    @Override
    public void setSlidePosition(double rotations) {
        slideMotor.setControl(slidePositionRequest.withPosition(rotations));
    }

    @Override
    public void stop() {
        rotatorMotor.stopMotor();
        slideMotor.stopMotor();
    }

    @Override
    public void zeroSlide() {
        slideMotor.setPosition(0.0);
    }
}
