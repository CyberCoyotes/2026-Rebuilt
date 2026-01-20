package frc.robot.subsystems.indexer2;

/**
 * IndexerIOSim - Simulation implementation of IndexerIO for testing without hardware.
 *
 * This class provides fake indexer data for testing robot logic without real motors/sensors.
 * Useful for:
 * - Testing indexer state machine logic
 * - Testing commands that use indexer
 * - Simulation mode
 * - Rapid prototyping at home
 *
 * FEATURES:
 * - Simulated motor speeds
 * - Simulated game piece detection
 * - Methods to control simulation state for testing
 */
public class IndexerIOSim implements IndexerIO {

    // ===== Simulated State =====
    private double floorMotorPercent = 0.0;
    private double feederMotorPercent = 0.0;
    private boolean gamePiecePresent = false;

    // ===== Simulation Parameters =====
    private static final double SIMULATED_TOF_DISTANCE_WITH_PIECE = 50.0;  // mm
    private static final double SIMULATED_TOF_DISTANCE_NO_PIECE = 500.0;    // mm
    private static final double MOTOR_VOLTAGE = 12.0;
    private static final double MOTOR_CURRENT_PER_PERCENT = 10.0;  // Amps per 100% speed
    private static final double MOTOR_TEMP = 35.0;  // Celsius

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Simulate floor motor
        inputs.floorVelocityRPS = floorMotorPercent * 10.0;  // Fake velocity
        inputs.floorAppliedVolts = floorMotorPercent * MOTOR_VOLTAGE;
        inputs.floorCurrentAmps = Math.abs(floorMotorPercent) * MOTOR_CURRENT_PER_PERCENT;
        inputs.floorTempCelsius = MOTOR_TEMP;

        // Simulate feeder motor
        inputs.feederVelocityRPS = feederMotorPercent * 10.0;
        inputs.feederAppliedVolts = feederMotorPercent * MOTOR_VOLTAGE;
        inputs.feederCurrentAmps = Math.abs(feederMotorPercent) * MOTOR_CURRENT_PER_PERCENT;
        inputs.feederTempCelsius = MOTOR_TEMP;

        // Simulate ToF sensor
        inputs.tofDistanceMM = gamePiecePresent ?
            SIMULATED_TOF_DISTANCE_WITH_PIECE :
            SIMULATED_TOF_DISTANCE_NO_PIECE;
        inputs.tofValid = true;
        inputs.gamePieceDetected = gamePiecePresent;
    }

    @Override
    public void setFloorMotor(double percent) {
        this.floorMotorPercent = percent;
    }

    @Override
    public void setFeederMotor(double percent) {
        this.feederMotorPercent = percent;
    }

    @Override
    public void stop() {
        this.floorMotorPercent = 0.0;
        this.feederMotorPercent = 0.0;
    }

    // ===== Simulation Control Methods =====

    /**
     * Simulates a game piece entering the indexer.
     * Call this to test state transitions when a piece is detected.
     */
    public void setGamePiecePresent(boolean present) {
        this.gamePiecePresent = present;
    }

    /**
     * Gets whether a game piece is simulated as present.
     */
    public boolean isGamePiecePresent() {
        return gamePiecePresent;
    }
}
