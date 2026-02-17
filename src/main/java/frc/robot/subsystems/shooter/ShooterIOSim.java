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
    private double targetHoodPosition = 0.0;
    private double currentHoodPosition = 0.0;

    // ===== Simulation Parameters =====
    private static final double FLYWHEEL_ACCELERATION_RPM_PER_CYCLE = 50.0;  // RPM gained per 20ms cycle
    private static final double HOOD_SPEED_ROT_PER_CYCLE = 0.1;  // Rotations per 20ms cycle
    private static final double MOTOR_VOLTAGE = 12.0;
    private static final double FLYWHEEL_CURRENT_PER_1000_RPM = 5.0;  // Amps per 1000 RPM
    private static final double HOOD_CURRENT = 2.0;  // Amps when moving

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
        if (currentHoodPosition < targetHoodPosition) {
            currentHoodPosition = Math.min(currentHoodPosition + HOOD_SPEED_ROT_PER_CYCLE,
                                        targetHoodPosition);
        } else if (currentHoodPosition > targetHoodPosition) {
            currentHoodPosition = Math.max(currentHoodPosition - HOOD_SPEED_ROT_PER_CYCLE,
                                        targetHoodPosition);
        }

        // Flywheel data (leader motor — matches ShooterIOHardware field names)
        double motorRPS = currentFlywheelRPM / 60.0;
        inputs.flywheelLeaderMotorRPM = currentFlywheelRPM;
        inputs.flywheelLeaderMotorRPS = motorRPS;
        inputs.flywheelMotorRPS = motorRPS;
        inputs.flywheelAppliedVolts = (currentFlywheelRPM != 0) ? MOTOR_VOLTAGE : 0.0;
        inputs.flywheelCurrentAmps = (Math.abs(currentFlywheelRPM) / 1000.0) * FLYWHEEL_CURRENT_PER_1000_RPM;

        // Hood data (raw motor rotations, matching hardware units)
        inputs.hoodPositionRotations = currentHoodPosition;
        inputs.hoodAngleDegrees = currentHoodPosition * 360.0;
        inputs.hoodAppliedVolts = (currentHoodPosition != targetHoodPosition) ? MOTOR_VOLTAGE : 0.0;
        inputs.hoodCurrentAmps = (currentHoodPosition != targetHoodPosition) ? HOOD_CURRENT : 0.0;

        // ThroughBore encoder sim (mirrors hood position, always "connected")
        inputs.hoodThroughBorePositionRotations = currentHoodPosition;
        inputs.hoodThroughBorePositionDegrees = currentHoodPosition * 360.0;
        inputs.hoodThroughBoreConnected = true;
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        this.targetFlywheelRPM = rpm;
    }

    @Override
    public void stopFlywheels() {
        this.targetFlywheelRPM = 0.0;
    }

    @Override
    public void setHoodPose(double rawPosition) {
        this.targetHoodPosition = Math.max(0, Math.min(10.0, rawPosition));  // Clamp to reasonable range
    }

    @Override
    public void stop() {
        this.targetFlywheelRPM = 0.0;
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
     * Instantly sets the hood to target position (for testing).
     * Bypasses realistic movement simulation.
     */
    public void setHoodInstant(double rotations) {
        this.currentHoodPosition = rotations;
        this.targetHoodPosition = rotations;
    }

    /**
     * Gets the current simulated flywheel velocity.
     */
    public double getCurrentFlywheelRPM() {
        return currentFlywheelRPM;
    }

    /**
     * Gets the current simulated hood position in rotations.
     */
    public double getCurrentHoodPosition() {
        return currentHoodPosition;
    }
}
