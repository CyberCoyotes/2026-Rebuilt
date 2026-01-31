package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * IndexerSubsystem - Moves game pieces from intake through the robot to the shooter.
 *
 * This subsystem uses the IO pattern for hardware abstraction:
 * - IndexerIO: Interface defining what hardware can do
 * - IndexerIOHardware: Real hardware implementation (motors + sensors)
 * - IndexerIOSim: Simulation for testing without hardware
 *
 * This pattern allows:
 * - Testing robot logic without hardware (simulation mode)
 * - Replaying logged matches with AdvantageKit
 * - Switching between different hardware implementations
 *
 * @see Constants.Indexer for hardware configuration
 * @see IndexerSubsystemBasic for a simpler direct-hardware approach (good for learning)
 */
public class IndexerSubsystem extends SubsystemBase {

    // ===== IO Layer =====
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    // ===== NetworkTables Publishers for Elastic Dashboard =====
    private final NetworkTable indexerTable;
    private final StringPublisher statePublisher;
    private final BooleanPublisher hasGamePiecePublisher;
    private final BooleanPublisher hopperFullPublisher;
    private final IntegerPublisher hopperCountPublisher;
    private final BooleanPublisher hopperAPublisher;
    private final BooleanPublisher hopperBPublisher;
    private final BooleanPublisher hopperCPublisher;
    private final DoublePublisher conveyorVelocityPublisher;
    private final DoublePublisher indexerVelocityPublisher;

    // ===== State Tracking =====
    private String currentState = "IDLE";

    /**
     * Creates a new IndexerSubsystem with the specified IO implementation.
     *
     * @param io The IndexerIO implementation (hardware or simulation)
     */
    public IndexerSubsystem(IndexerIO io) {
        this.io = io;

        // Initialize NetworkTables publishers for Elastic dashboard
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        indexerTable = inst.getTable("Indexer");

        statePublisher = indexerTable.getStringTopic("State").publish();
        hasGamePiecePublisher = indexerTable.getBooleanTopic("HasGamePiece").publish();
        hopperFullPublisher = indexerTable.getBooleanTopic("HopperFull").publish();
        hopperCountPublisher = indexerTable.getIntegerTopic("HopperCount").publish();
        hopperAPublisher = indexerTable.getBooleanTopic("HopperA").publish();
        hopperBPublisher = indexerTable.getBooleanTopic("HopperB").publish();
        hopperCPublisher = indexerTable.getBooleanTopic("HopperC").publish();
        conveyorVelocityPublisher = indexerTable.getDoubleTopic("ConveyorVelocityRPS").publish();
        indexerVelocityPublisher = indexerTable.getDoubleTopic("IndexerVelocityRPS").publish();
    }

    @Override
    public void periodic() {
        // Update inputs from hardware/simulation every cycle (20ms)
        io.updateInputs(inputs);

        // Log all inputs for AdvantageKit replay
        Logger.processInputs("Indexer", inputs);

        // Publish to NetworkTables for Elastic dashboard
        publishTelemetry();
    }

    /**
     * Publishes indexer telemetry to NetworkTables for Elastic dashboard.
     */
    private void publishTelemetry() {
        statePublisher.set(currentState);
        hasGamePiecePublisher.set(isGamePieceAtIndexer());
        hopperFullPublisher.set(isHopperFull());
        hopperCountPublisher.set(getHopperGamePieceCount());
        hopperAPublisher.set(inputs.hopperADetected);
        hopperBPublisher.set(inputs.hopperBDetected);
        hopperCPublisher.set(inputs.hopperCDetected);
        conveyorVelocityPublisher.set(inputs.conveyorVelocityRPS);
        indexerVelocityPublisher.set(inputs.indexerVelocityRPS);
    }

    /**
     * Sets the current state for dashboard display.
     *
     * @param state State string (e.g., "IDLE", "FEEDING", "INTAKING")
     */
    public void setState(String state) {
        this.currentState = state;
    }

    // ========== Motor Control Methods ==========

    /**
     * Sets the conveyor motor speed (moves pieces toward indexer).
     *
     * @param percent Motor speed from -1.0 to 1.0 (+ = toward indexer)
     */
    public void setConveyorMotorSpeed(double percent) {
       io.setConveyorMotor(percent); 
    }

    /**
     * Sets the indexer motor speed (feeds pieces to shooter).
     *
     * @param percent Motor speed from -1.0 to 1.0 (+ = toward shooter)
     */
    public void setIndexerMotorSpeed(double percent) {
        io.setIndexerMotor(percent);
    }

    /**
     * Stops both motors immediately.
     */
    public void stop() {
        io.stop();
    }

    // ========== Sensor Methods ==========

    /**
     * Returns true if a game piece is detected at the indexer (ready to shoot).
     */
    public boolean isGamePieceAtIndexer() {
        return inputs.gamePieceDetected;
    }

    /**
     * Returns true if a game piece is detected at hopper position A.
     */
    public boolean isGamePieceAtHopperA() {
        return inputs.hopperADetected;
    }

    /**
     * Returns true if a game piece is detected at hopper position B.
     */
    public boolean isGamePieceAtHopperB() {
        return inputs.hopperBDetected;
    }

    /**
     * Returns true if a game piece is detected at hopper position C.
     */
    public boolean isGamePieceAtHopperC() {
        return inputs.hopperCDetected;
    }

    /**
     * Returns true if any hopper sensor detects a game piece.
     */
    public boolean isGamePieceInHopper() {
        return inputs.hopperADetected || inputs.hopperBDetected || inputs.hopperCDetected;
    }

    /**
     * Returns the count of game pieces currently detected in the hopper (0-3).
     */
    public int getHopperGamePieceCount() {
        int count = 0;
        if (inputs.hopperADetected) count++;
        if (inputs.hopperBDetected) count++;
        if (inputs.hopperCDetected) count++;
        return count;
    }

    /**
     * Returns true if the hopper is full (all 3 positions have game pieces).
     */
    public boolean isHopperFull() {
        return inputs.hopperADetected && inputs.hopperBDetected && inputs.hopperCDetected;
    }

    // ========== Telemetry Getters ==========

    /** Gets conveyor motor velocity in rotations per second */
    public double getConveyorVelocityRPS() {
        return inputs.conveyorVelocityRPS;
    }

    /** Gets indexer motor velocity in rotations per second */
    public double getIndexerVelocityRPS() {
        return inputs.indexerVelocityRPS;
    }

    /** Gets conveyor motor current draw in amps */
    public double getconveyorCurrentAmps() {
        return inputs.conveyorCurrentAmps;
    }

    /** Gets indexer motor current draw in amps */
    public double getIndexerCurrentAmps() {
        return inputs.indexerCurrentAmps;
    }

    /** Gets the raw ToF distance at the indexer in mm */
    public double getIndexerTofDistanceMM() {
        return inputs.tofDistanceMM;
    }
}
