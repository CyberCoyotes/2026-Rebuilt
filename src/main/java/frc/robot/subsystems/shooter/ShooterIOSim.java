package frc.robot.subsystems.shooter;

/**
 * ShooterIOSim - Simulation implementation of ShooterIO for testing without hardware.
 *
 * Provides fake shooter data for testing robot logic without real motors.
 * Useful for: testing state machine logic, testing commands, simulation mode,
 * and rapid prototyping at home.
 */
public class ShooterIOSim implements ShooterIO {

    // ===== Simulated State =====
    private double targetFlywheelRPM = 0.0;
    private double currentFlywheelRPM = 0.0;
    private double targetHoodPosition = 0.0;
    private double currentHoodPosition = 0.0;

    // ===== Simulation Parameters =====
    private static final double FLYWHEEL_ACCELERATION_RPM_PER_CYCLE = 50.0;
    private static final double HOOD_SPEED_ROT_PER_CYCLE = 0.1;
    private static final double MOTOR_VOLTAGE = 12.0;
    private static final double FLYWHEEL_CURRENT_PER_1000_RPM = 5.0;
    private static final double HOOD_CURRENT = 2.0;

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

        // Fast fields — control-critical
        double motorRPS = currentFlywheelRPM / 60.0;
        inputs.flywheelLeaderMotorRPM = currentFlywheelRPM;
        inputs.flywheelLeaderMotorRPS = motorRPS;
        inputs.flywheelAppliedVolts = (currentFlywheelRPM != 0) ? MOTOR_VOLTAGE : 0.0;
        inputs.hoodPositionRotations = currentHoodPosition;
    }

    @Override
    public void updateSlowInputs(ShooterIOInputs inputs) {
        // Slow fields — diagnostics
        inputs.flywheelCurrentAmps = Math.abs(currentFlywheelRPM) / 1000.0 * FLYWHEEL_CURRENT_PER_1000_RPM;
        inputs.hoodAppliedVolts = (currentHoodPosition != targetHoodPosition) ? MOTOR_VOLTAGE * 0.5 : 0.0;
        inputs.hoodCurrentAmps = (currentHoodPosition != targetHoodPosition) ? HOOD_CURRENT : 0.0;
        inputs.hoodAngleDegrees = currentHoodPosition * 360.0;

        // Simulated ThroughBore — always connected in sim
        inputs.hoodThroughBorePositionRotations = currentHoodPosition;
        inputs.hoodThroughBorePositionDegrees = currentHoodPosition * 360.0;
        inputs.hoodThroughBoreConnected = true;
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        targetFlywheelRPM = rpm;
    }

    @Override
    public void stopFlywheels() {
        targetFlywheelRPM = 0.0;
    }

    @Override
    public void setHoodPose(double rawPosition) {
        targetHoodPosition = rawPosition;
    }

    @Override
    public void setHoodVoltage(double volts) {
        // Sim: approximate voltage-based movement
        targetHoodPosition += volts * 0.01;
    }

    @Override
    public void stop() {
        targetFlywheelRPM = 0.0;
        targetHoodPosition = currentHoodPosition; // stop hood where it is
    }
}