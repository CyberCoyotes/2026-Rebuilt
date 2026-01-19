package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * IndexerSubsystem - Manages game piece movement from intake to shooter.
 *
 * HARDWARE:
 * - Floor motor: Moves game pieces along the hopper floor toward the feeder
 * - Feeder motor: Grabs pieces from hopper and feeds them into the shooter
 * - ToF sensor: Detects when a game piece is staged and ready to shoot
 *
 * STATE MACHINE:
 * - IDLE: All motors off, waiting for commands
 * - FLOOR_TRANSPORT: Floor motor on, moving piece toward feeder
 * - FEEDING: Both motors on, actively feeding piece to shooter
 * - EJECTING: Both motors reversed, clearing jammed pieces
 *
 * TYPICAL FLOW:
 * 1. Intake picks up game piece
 * 2. FLOOR_TRANSPORT: Floor motor moves piece toward feeder
 * 3. ToF sensor detects piece → transition to IDLE (piece staged)
 * 4. When shooter ready → FEEDING (shoot the piece)
 * 5. ToF sensor clears → back to IDLE
 *
 * AUTHOR: @Joel-Trumpet-67
 */
public class IndexerSubsystem extends SubsystemBase {

    // ===== Hardware Interface =====
    private final IndexerIO io;
    private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();

    // ===== State Machine =====
    /**
     * IndexerState - Defines what the indexer is actively doing.
     *
     * States represent ACTIONS, not conditions.
     * Use hasGamePiece() to check sensor status separately.
     */
    public enum IndexerState {
        /** Motors off, waiting for next action */
        IDLE,

        /** Floor motor running, moving piece toward feeder */
        FLOOR_TRANSPORT,

        /** Floor + feeder motors running, feeding piece to shooter */
        FEEDING,

        /** Floor + feeder reversed, ejecting jammed piece */
        EJECTING
    }

    private IndexerState currentState = IndexerState.IDLE;
    private IndexerState previousState = IndexerState.IDLE;

    // ===== Motor Speeds =====
    private static final double FLOOR_SPEED = 0.5;    // TODO: Tune on robot
    private static final double FEEDER_SPEED = 0.7;   // TODO: Tune on robot
    private static final double EJECT_SPEED = -0.4;   // TODO: Tune on robot

    /**
     * Creates a new IndexerSubsystem.
     *
     * @param io The IndexerIO hardware interface implementation
     */
    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        // Execute current state
        runState();

        // Log telemetry
        logTelemetry();
    }

    // =========================================================================
    // STATE MACHINE
    // =========================================================================

    /**
     * Executes the current state by setting motor speeds.
     * Called automatically in periodic().
     */
    private void runState() {
        switch (currentState) {
            case IDLE:
                io.stop();
                break;

            case FLOOR_TRANSPORT:
                io.setFloorMotor(FLOOR_SPEED);
                io.setFeederMotor(0.0);
                break;

            case FEEDING:
                io.setFloorMotor(FLOOR_SPEED);
                io.setFeederMotor(FEEDER_SPEED);
                break;

            case EJECTING:
                io.setFloorMotor(EJECT_SPEED);
                io.setFeederMotor(EJECT_SPEED);
                break;
        }
    }

    /**
     * Sets the indexer state and logs the transition.
     *
     * @param newState The state to transition to
     */
    public void setState(IndexerState newState) {
        if (currentState != newState) {
            previousState = currentState;
            currentState = newState;

            // Log state transition
            Logger.recordOutput("Indexer/StateTransition",
                previousState.name() + " -> " + currentState.name());
        }
    }

    // =========================================================================
    // PUBLIC API - State Queries
    // =========================================================================

    /**
     * Gets the current indexer state.
     *
     * @return Current state
     */
    public IndexerState getState() {
        return currentState;
    }

    /**
     * Checks if a game piece is detected by the ToF sensor.
     * This is a SENSOR STATUS, not a state.
     *
     * Use this to:
     * - Know when to stop FLOOR_TRANSPORT (piece staged)
     * - Know when piece has left (after FEEDING)
     * - Coordinate with shooter (don't feed if no piece)
     *
     * @return true if game piece is detected
     */
    public boolean hasGamePiece() {
        return inputs.gamePieceDetected;
    }

    /**
     * Checks if a game piece is staged and ready to shoot.
     * Equivalent to: IDLE state AND sensor detects piece.
     *
     * @return true if ready to shoot
     */
    public boolean isReadyToFeed() {
        return currentState == IndexerState.IDLE && hasGamePiece();
    }

    // =========================================================================
    // PUBLIC API - State Commands
    // =========================================================================

    /**
     * Stops all indexer motors and returns to IDLE.
     */
    public void stop() {
        setState(IndexerState.IDLE);
    }

    /**
     * Starts transporting a game piece from intake to staging position.
     * Floor motor runs until ToF sensor detects piece.
     *
     * Typical usage: Call this when intake picks up a piece.
     */
    public void startTransport() {
        setState(IndexerState.FLOOR_TRANSPORT);
    }

    /**
     * Starts feeding game piece to shooter.
     * Both floor and feeder motors run.
     *
     * Typical usage: Call this when shooter is ready and spun up.
     */
    public void startFeeding() {
        setState(IndexerState.FEEDING);
    }

    /**
     * Starts ejecting game piece (reverse all motors).
     * Used to clear jams or reject bad pieces.
     */
    public void startEject() {
        setState(IndexerState.EJECTING);
    }

    // =========================================================================
    // TELEMETRY
    // =========================================================================

    /**
     * Logs comprehensive telemetry to SmartDashboard and AdvantageKit.
     */
    private void logTelemetry() {
        // State information
        SmartDashboard.putString("Indexer/State", currentState.name());
        SmartDashboard.putBoolean("Indexer/HasGamePiece", hasGamePiece());
        SmartDashboard.putBoolean("Indexer/ReadyToFeed", isReadyToFeed());

        // ToF sensor
        SmartDashboard.putNumber("Indexer/ToF_Distance_mm", inputs.tofDistanceMM);
        SmartDashboard.putBoolean("Indexer/ToF_Valid", inputs.tofValid);

        // Motor telemetry
        SmartDashboard.putNumber("Indexer/FloorCurrent_A", inputs.floorCurrentAmps);
        SmartDashboard.putNumber("Indexer/FeederCurrent_A", inputs.feederCurrentAmps);

        // AdvantageKit logging
        Logger.recordOutput("Indexer/State", currentState.name());
        Logger.recordOutput("Indexer/HasGamePiece", hasGamePiece());
        Logger.recordOutput("Indexer/ReadyToFeed", isReadyToFeed());
    }
}
