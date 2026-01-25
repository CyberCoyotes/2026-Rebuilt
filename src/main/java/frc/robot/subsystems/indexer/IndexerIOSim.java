package frc.robot.subsystems.indexer;

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
 * - Simulated game piece detection at indexer
 * - Simulated game piece detection at 3 hopper positions
 * - Methods to control simulation state for testing
 *
 * @see IndexerSubsystemBasic for a simpler direct-hardware approach (good for learning)
 */
public class IndexerIOSim implements IndexerIO {

    // ===== Simulated State =====
    private double conveyorMotorPercent = 0.0;
    private double indexerMotorPercent = 0.0;

    // Game piece presence simulation
    private boolean indexerGamePiecePresent = false;
    private boolean hopperAGamePiecePresent = false;
    private boolean hopperBGamePiecePresent = false;
    private boolean hopperCGamePiecePresent = false;

    // ===== Simulation Parameters =====
    private static final double SIMULATED_TOF_DISTANCE_WITH_PIECE = 50.0;   // mm
    private static final double SIMULATED_TOF_DISTANCE_NO_PIECE = 500.0;    // mm
    private static final double MOTOR_VOLTAGE = 12.0;
    private static final double MOTOR_CURRENT_PER_PERCENT = 10.0;  // Amps per 100% speed
    private static final double MOTOR_TEMP = 35.0;  // Celsius

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Simulate conveyor motor
        inputs.conveyorVelocityRPS = conveyorMotorPercent * 10.0;  // Fake velocity
        inputs.conveyorAppliedVolts = conveyorMotorPercent * MOTOR_VOLTAGE;
        inputs.conveyorCurrentAmps = Math.abs(conveyorMotorPercent) * MOTOR_CURRENT_PER_PERCENT;
        inputs.conveyorTempCelsius = MOTOR_TEMP;

        // Simulate indexer motor
        inputs.indexerVelocityRPS = indexerMotorPercent * 10.0;
        inputs.indexerAppliedVolts = indexerMotorPercent * MOTOR_VOLTAGE;
        inputs.indexerCurrentAmps = Math.abs(indexerMotorPercent) * MOTOR_CURRENT_PER_PERCENT;
        inputs.indexerTempCelsius = MOTOR_TEMP;

        // Simulate indexer ToF sensor
        inputs.tofDistanceMM = indexerGamePiecePresent ?
            SIMULATED_TOF_DISTANCE_WITH_PIECE :
            SIMULATED_TOF_DISTANCE_NO_PIECE;
        inputs.tofValid = true;
        inputs.gamePieceDetected = indexerGamePiecePresent;

        // Simulate hopper A ToF sensor
        inputs.hopperADistanceMM = hopperAGamePiecePresent ?
            SIMULATED_TOF_DISTANCE_WITH_PIECE :
            SIMULATED_TOF_DISTANCE_NO_PIECE;
        inputs.hopperAValid = true;
        inputs.hopperADetected = hopperAGamePiecePresent;

        // Simulate hopper B ToF sensor
        inputs.hopperBDistanceMM = hopperBGamePiecePresent ?
            SIMULATED_TOF_DISTANCE_WITH_PIECE :
            SIMULATED_TOF_DISTANCE_NO_PIECE;
        inputs.hopperBValid = true;
        inputs.hopperBDetected = hopperBGamePiecePresent;

        // Simulate hopper C ToF sensor
        inputs.hopperCDistanceMM = hopperCGamePiecePresent ?
            SIMULATED_TOF_DISTANCE_WITH_PIECE :
            SIMULATED_TOF_DISTANCE_NO_PIECE;
        inputs.hopperCValid = true;
        inputs.hopperCDetected = hopperCGamePiecePresent;
    }

    @Override
    public void setConveyorMotor(double percent) {
        this.conveyorMotorPercent = percent;
    }

    @Override
    public void setIndexerMotor(double percent) {
        this.indexerMotorPercent = percent;
    }

    @Override
    public void stop() {
        this.conveyorMotorPercent = 0.0;
        this.indexerMotorPercent = 0.0;
    }

    // ===== Simulation Control Methods =====

    /**
     * Simulates a game piece at the indexer position (ready to shoot).
     * Call this to test state transitions when a piece is ready to fire.
     *
     * @param present true if game piece should be simulated as present
     */
    public void setFeederGamePiecePresent(boolean present) {
        this.indexerGamePiecePresent = present;
    }

    /**
     * Gets whether a game piece is simulated as present at the indexer.
     */
    public boolean isFeederGamePiecePresent() {
        return indexerGamePiecePresent;
    }

    /**
     * Simulates a game piece at hopper position A.
     *
     * @param present true if game piece should be simulated as present
     */
    public void setHopperAGamePiecePresent(boolean present) {
        this.hopperAGamePiecePresent = present;
    }

    /**
     * Gets whether a game piece is simulated as present at hopper position A.
     */
    public boolean isHopperAGamePiecePresent() {
        return hopperAGamePiecePresent;
    }

    /**
     * Simulates a game piece at hopper position B.
     *
     * @param present true if game piece should be simulated as present
     */
    public void setHopperBGamePiecePresent(boolean present) {
        this.hopperBGamePiecePresent = present;
    }

    /**
     * Gets whether a game piece is simulated as present at hopper position B.
     */
    public boolean isHopperBGamePiecePresent() {
        return hopperBGamePiecePresent;
    }

    /**
     * Simulates a game piece at hopper position C.
     *
     * @param present true if game piece should be simulated as present
     */
    public void setHopperCGamePiecePresent(boolean present) {
        this.hopperCGamePiecePresent = present;
    }

    /**
     * Gets whether a game piece is simulated as present at hopper position C.
     */
    public boolean isHopperCGamePiecePresent() {
        return hopperCGamePiecePresent;
    }

    /**
     * Convenience method to set all hopper positions at once.
     * Useful for simulating a full or empty hopper.
     *
     * @param a game piece present at position A
     * @param b game piece present at position B
     * @param c game piece present at position C
     */
    public void setHopperGamePiecesPresent(boolean a, boolean b, boolean c) {
        this.hopperAGamePiecePresent = a;
        this.hopperBGamePiecePresent = b;
        this.hopperCGamePiecePresent = c;
    }

    /**
     * Simulates a full hopper (all 3 positions have game pieces).
     */
    public void simulateFullHopper() {
        setHopperGamePiecesPresent(true, true, true);
    }

    /**
     * Simulates an empty hopper (no game pieces).
     */
    public void simulateEmptyHopper() {
        setHopperGamePiecesPresent(false, false, false);
        setFeederGamePiecePresent(false);
    }

    // ===== Backwards Compatibility =====
    // These methods maintain compatibility with old code that used the single-sensor API

    /**
     * @deprecated Use setFeederGamePiecePresent instead
     */
    @Deprecated
    public void setGamePiecePresent(boolean present) {
        setFeederGamePiecePresent(present);
    }

    /**
     * @deprecated Use isFeederGamePiecePresent instead
     */
    @Deprecated
    public boolean isGamePiecePresent() {
        return isFeederGamePiecePresent();
    }
}
