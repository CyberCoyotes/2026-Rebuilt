package frc.robot.subsystems.indexer;

import frc.robot.Constants;

/**
 * IndexerIOSim - Simulation implementation of IndexerIO.
 *
 * Matches the current IndexerIO interface exactly:
 * - Conveyor: tracks commanded voltage, simulates velocity
 * - Kicker: tracks commanded voltage, simulates velocity
 * - Chute CANrange: simulated distance, manually controllable for testing
 *
 * To test the full feed sequence in sim, call setChuteDetected(true) to
 * simulate a game piece being present in the chute.
 */
public class IndexerIOSim implements IndexerIO {

    // Commanded voltages
    private double conveyorVolts = 0.0;
    private double kickerVolts   = 0.0;

    // Simulated chute sensor state — manually set for testing
    private boolean chuteDetected    = false;
    private double  chuteDistanceMeters = Constants.Indexer.CHUTE_MAX_DISTANCE; // far = no detection

    // Simulated motor characteristics
    private static final double NOMINAL_VOLTAGE  = 12.0;
    private static final double RPS_PER_VOLT     = 10.0 / NOMINAL_VOLTAGE; // ~10 RPS at full voltage
    private static final double CURRENT_PER_VOLT = 0.5;                    // ~6A at 12V

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Conveyor
        inputs.conveyorVelocityRPS = conveyorVolts * RPS_PER_VOLT;
        inputs.conveyorCurrentAmps = Math.abs(conveyorVolts) * CURRENT_PER_VOLT;

        // Kicker
        inputs.kickerLeadVelocityRPS  = kickerVolts * RPS_PER_VOLT;
        inputs.kickerLeadCurrentAmps  = Math.abs(kickerVolts) * CURRENT_PER_VOLT;
        inputs.kickerFollowCurrentAmps = Math.abs(kickerVolts) * CURRENT_PER_VOLT;

        // Chute CANrange
        inputs.chuteDistanceMeters = chuteDistanceMeters;
        inputs.chuteDetected       = chuteDetected;
    }

    @Override
    public void setConveyorMotor(double volts) {
        conveyorVolts = volts;
    }

    @Override
    public void setKickerMotorVolts(double volts) {
        kickerVolts = volts;
    }

    @Override
    public void stop() {
        conveyorVolts = 0.0;
        kickerVolts   = 0.0;
    }

    // =========================================================================
    // Simulation Control — call these from tests or sim periodic to inject state
    // =========================================================================

    /** Simulates a game piece present in the chute. */
    public void setChuteDetected(boolean detected) {
        this.chuteDetected = detected;
        this.chuteDistanceMeters = detected
            ? Constants.Indexer.FUEL_DETECTION_DISTANCE * 0.5  // well inside threshold
            : Constants.Indexer.CHUTE_MAX_DISTANCE;             // far away = no detection
    }
}