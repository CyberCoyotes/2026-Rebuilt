package frc.robot.subsystems.indexer;

/**
 * IndexerIOSim - Simulation implementation of IndexerIO.
 *
 * Conveyor and kicker are modeled as first-order velocity filters.
 * The chute CANrange sensor always reports no game piece; inject one via
 * simulateGamePieceInChute() to test detection-dependent logic.
 */
public class IndexerIOSim implements IndexerIO {

    // Motor state
    private double conveyorTargetRPS = 0.0;
    private double conveyorVelocityRPS = 0.0;
    private double kickerTargetRPS = 0.0;
    private double kickerVelocityRPS = 0.0;

    // Sensor state — start with no game piece present
    private boolean chuteDetected = false;

    // First-order time constant for both motors
    private static final double MOTOR_TC_SEC = 0.1;
    // Rough RPS at 12V for voltage-based commands
    private static final double MAX_RPS_AT_12V = 50.0;

    private static final double DT = 0.02;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        double alpha = DT / MOTOR_TC_SEC;
        conveyorVelocityRPS += alpha * (conveyorTargetRPS - conveyorVelocityRPS);
        kickerVelocityRPS += alpha * (kickerTargetRPS - kickerVelocityRPS);

        inputs.conveyorVelocityRPS = conveyorVelocityRPS;
        inputs.conveyorCurrentAmps = Math.abs(conveyorVelocityRPS) * 0.5;
        inputs.kickerLeadVelocityRPS = kickerVelocityRPS;
        inputs.kickerLeadCurrentAmps = Math.abs(kickerVelocityRPS) * 0.5;
        inputs.kickerFollowCurrentAmps = inputs.kickerLeadCurrentAmps;
        inputs.chuteDistanceMeters = chuteDetected ? 0.05 : 1.0;
        inputs.chuteDetected = chuteDetected;
    }

    @Override
    public void setConveyorVelocity(double rps) {
        conveyorTargetRPS = rps;
    }

    @Override
    public void setConveyorMotor(double volts) {
        conveyorTargetRPS = (volts / 12.0) * MAX_RPS_AT_12V;
    }

    @Override
    public void setKickerVelocity(double rps) {
        kickerTargetRPS = rps;
    }

    @Override
    public void setKickerMotorVolts(double volts) {
        kickerTargetRPS = (volts / 12.0) * MAX_RPS_AT_12V;
    }

    @Override
    public void stop() {
        conveyorTargetRPS = 0.0;
        kickerTargetRPS = 0.0;
    }

    /** Call from test code or SmartDashboard to inject a simulated game piece. */
    public void simulateGamePieceInChute(boolean present) {
        chuteDetected = present;
    }
}
