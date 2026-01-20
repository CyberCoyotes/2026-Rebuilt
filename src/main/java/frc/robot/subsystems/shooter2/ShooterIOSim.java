package frc.robot.subsystems.shooter;

/**
 * ShooterIOSim - Simulation implementation of ShooterIO for testing without hardware.
 *
 * This class provides fake shooter data for testing robot logic without real motors.
 * Useful for:
 * - Testing shooter state machine logic
 * - Testing commands that use shooter
 * - Simulation mode
 * - Rapid prototyping at home
 *
 * FEATURES:
 * - Simulated flywheel velocity with realistic acceleration
 * - Simulated hood position
 * - Methods to control simulation state for testing
 */
public class ShooterIOSim implements ShooterIO {

    // ===== Simulated State =====
    private double targetFlywheelRPM = 0.0;
    private double currentFlywheelRPM = 0.0;
    private double targetHoodAngle = 0.0;
    private double currentHoodAngle = 0.0;
    private double counterWheelRPM = 0.0;

    // ===== Simulation Parameters =====
    private static final double FLYWHEEL_ACCELERATION_RPM_PER_CYCLE = 50.0;  // RPM gained per 20ms cycle
    private static final double HOOD_SPEED_DEG_PER_CYCLE = 2.0;  // Degrees per 20ms cycle
    private static final double MOTOR_VOLTAGE = 12.0;
    private static final double FLYWHEEL_CURRENT_PER_1000_RPM = 5.0;  // Amps per 1000 RPM
    private static final double HOOD_CURRENT = 2.0;  // Amps when moving
    private static final double MOTOR_TEMP = 40.0;  // Celsius

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Simulate flywheel acceleration/deceleration
        if (currentFlywheelRPM < targetFlywheelRPM) {
            currentFlywheelRPM = Math.min(currentFlywheelRPM + FLYWHEEL_ACCELERATION_RPM_PER_CYCLE,
                                          targetFlywheelRPM);
        } else if (currentFlywheelRPM > targetFlywheelRPM) {
            currentFlywheelRPM = Math.max(currentFlywheelRPM - FLYWHEEL_ACCELERATION_RPM_PER_CYCLE,
                                          targetFlywheelRPM);
        }

        // Simulate hood movement
        if (currentHoodAngle < targetHoodAngle) {
            currentHoodAngle = Math.min(currentHoodAngle + HOOD_SPEED_DEG_PER_CYCLE,
                                        targetHoodAngle);
        } else if (currentHoodAngle > targetHoodAngle) {
            currentHoodAngle = Math.max(currentHoodAngle - HOOD_SPEED_DEG_PER_CYCLE,
                                        targetHoodAngle);
        }

        // Main flywheel data (averaged - all the same in sim)
        inputs.flywheelVelocityRPM = currentFlywheelRPM;
        inputs.flywheelAppliedVolts = (currentFlywheelRPM > 0) ? MOTOR_VOLTAGE : 0.0;
        inputs.flywheelCurrentAmps = (currentFlywheelRPM / 1000.0) * FLYWHEEL_CURRENT_PER_1000_RPM;
        inputs.flywheelTempCelsius = MOTOR_TEMP;

        // Individual flywheel data (all the same in sim)
        inputs.flywheelAVelocityRPM = currentFlywheelRPM;
        inputs.flywheelBVelocityRPM = currentFlywheelRPM;
        inputs.flywheelCVelocityRPM = currentFlywheelRPM;

        // Hood data
        inputs.hoodAngleDegrees = currentHoodAngle;
        inputs.hoodAppliedVolts = (currentHoodAngle != targetHoodAngle) ? MOTOR_VOLTAGE : 0.0;
        inputs.hoodCurrentAmps = (currentHoodAngle != targetHoodAngle) ? HOOD_CURRENT : 0.0;

        // Counter-wheel data
        inputs.counterWheelVelocityRPM = counterWheelRPM;
        inputs.counterWheelAppliedVolts = (counterWheelRPM > 0) ? MOTOR_VOLTAGE : 0.0;
        inputs.counterWheelCurrentAmps = Math.abs(counterWheelRPM) / 100.0;  // Fake current
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        this.targetFlywheelRPM = rpm;
    }

    @Override
    public void setHoodAngle(double degrees) {
        this.targetHoodAngle = Math.max(0, Math.min(90, degrees));  // Clamp 0-90 degrees
    }

    @Override
    public void setCounterWheelVelocity(double rpm) {
        this.counterWheelRPM = rpm;
    }

    @Override
    public void stop() {
        this.targetFlywheelRPM = 0.0;
        this.counterWheelRPM = 0.0;
        // Hood stays at current position when stopped
    }

    // ===== Simulation Control Methods =====

    /**
     * Instantly sets the flywheel to target velocity (for testing).
     * Bypasses realistic acceleration simulation.
     */
    public void setFlywheelInstant(double rpm) {
        this.currentFlywheelRPM = rpm;
        this.targetFlywheelRPM = rpm;
    }

    /**
     * Instantly sets the hood to target angle (for testing).
     * Bypasses realistic movement simulation.
     */
    public void setHoodInstant(double degrees) {
        this.currentHoodAngle = degrees;
        this.targetHoodAngle = degrees;
    }

    /**
     * Gets the current simulated flywheel velocity.
     */
    public double getCurrentFlywheelRPM() {
        return currentFlywheelRPM;
    }

    /**
     * Gets the current simulated hood angle.
     */
    public double getCurrentHoodAngle() {
        return currentHoodAngle;
    }
}
