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
 * - Simulated game piece detection at feeder
 * - Simulated game piece detection at 3 hopper positions
 * - Methods to control simulation state for testing
 *
 * @see IndexerSubsystemBasic for a simpler direct-hardware approach (good for learning)
 */
public class IndexerIOSim implements IndexerIO {

    // ===== Simulated State =====
    private double floorMotorPercent = 0.0;
    private double feederMotorPercent = 0.0;

    // Game piece presence simulation
    private boolean feederGamePiecePresent = false;
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

        // Simulate feeder ToF sensor
        inputs.tofDistanceMM = feederGamePiecePresent ?
            SIMULATED_TOF_DISTANCE_WITH_PIECE :
            SIMULATED_TOF_DISTANCE_NO_PIECE;
        inputs.tofValid = true;
        inputs.gamePieceDetected = feederGamePiecePresent;

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
     * Simulates a game piece at the feeder position (ready to shoot).
     * Call this to test state transitions when a piece is ready to fire.
     *
     * @param present true if game piece should be simulated as present
     */
    public void setFeederGamePiecePresent(boolean present) {
        this.feederGamePiecePresent = present;
    }

    /**
     * Gets whether a game piece is simulated as present at the feeder.
     */
    public boolean isFeederGamePiecePresent() {
        return feederGamePiecePresent;
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
