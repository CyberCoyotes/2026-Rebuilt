package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

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
    private final DoublePublisher conveyorVelocityPublisher;
    private final DoublePublisher indexerVelocityPublisher;

    // ===== Jam Detection Telemetry Publishers =====
    private final BooleanPublisher hopperJammedPublisher;
    private final BooleanPublisher indexerJammedPublisher;
    private final DoublePublisher conveyorCurrentPublisher;
    private final DoublePublisher indexerCurrentPublisher;

    // ===== Motor Voltage Constants =====
    public static final double CONVEYOR_FORWARD_VOLTAGE = 8.0; // TODO: Tune conveyor on robot
    public static final double CONVEYOR_REVERSE_VOLTAGE = -8.0;

    public static final double INDEXER_FORWARD_VOLTAGE = 5.0; // TODO: Tune indexer on robot
    public static final double INDEXER_REVERSE_VOLTAGE = -5.0;

    // ===== State Tracking =====
    private String currentState = "IDLE";

    /**
     * Creates a new IndexerSubsystem with the specified IO implementation.
     *
     * @param io The IndexerIO implementation (hardware or simulation)
     */
    public IndexerSubsystem(IndexerIO io) {
        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        indexerTable = inst.getTable("Indexer");

        statePublisher = indexerTable.getStringTopic("State").publish();
        hasGamePiecePublisher = indexerTable.getBooleanTopic("HasGamePiece").publish();
        hopperFullPublisher = indexerTable.getBooleanTopic("HopperFull").publish();
        hopperCountPublisher = indexerTable.getIntegerTopic("HopperCount").publish();
        hopperAPublisher = indexerTable.getBooleanTopic("HopperA").publish();
        hopperBPublisher = indexerTable.getBooleanTopic("HopperB").publish();
        conveyorVelocityPublisher = indexerTable.getDoubleTopic("ConveyorVelocityRPS").publish();
        indexerVelocityPublisher = indexerTable.getDoubleTopic("IndexerVelocityRPS").publish();

        hopperJammedPublisher = indexerTable.getBooleanTopic("HopperJammed").publish();
        indexerJammedPublisher = indexerTable.getBooleanTopic("IndexerJammed").publish();
        conveyorCurrentPublisher = indexerTable.getDoubleTopic("ConveyorCurrentAmps").publish();
        indexerCurrentPublisher = indexerTable.getDoubleTopic("IndexerCurrentAmps").publish();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
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
        conveyorVelocityPublisher.set(inputs.conveyorVelocityRPS);
        indexerVelocityPublisher.set(inputs.indexerVelocityRPS);

        hopperJammedPublisher.set(isHopperJammed());
        indexerJammedPublisher.set(isIndexerJammed());
        conveyorCurrentPublisher.set(inputs.conveyorCurrentAmps);
        indexerCurrentPublisher.set(inputs.indexerCurrentAmps);
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
     * Sets the conveyor motor voltage (moves pieces toward indexer).
     *
     * @param volts Motor voltage (+ = toward indexer, - = reverse)
     */
    public void setConveyorMotorVolts(double volts) {
        io.setConveyorMotor(volts);
    }

    public void conveyorForward() {
        io.setConveyorMotor(CONVEYOR_FORWARD_VOLTAGE);
    }

    public void conveyorReverse() {
        io.setConveyorMotor(CONVEYOR_REVERSE_VOLTAGE);
    }

    public void conveyorStop() {
        io.setConveyorMotor(0.0);
    }

    /**
     * Sets the indexer motor voltage (feeds pieces to shooter).
     *
     * @param volts Motor voltage (+ = toward shooter, - = reverse)
     */
    public void setIndexerMotorVolts(double volts) {
        io.setIndexerMotor(volts);
    }

    public void indexerForward() {
        io.setIndexerMotor(INDEXER_FORWARD_VOLTAGE);
    }

    public void indexerReverse() {
        io.setIndexerMotor(INDEXER_REVERSE_VOLTAGE);
    }

    public void indexerStop() {
        io.setIndexerMotor(0.0);
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
     * Returns true if any hopper sensor detects a game piece.
     */
    public boolean isGamePieceInHopper() {
        return inputs.hopperADetected || inputs.hopperBDetected;
    }

    /**
     * Returns the count of game pieces currently detected in the hopper (0-2).
     */
    public int getHopperGamePieceCount() {
        int count = 0;
        if (inputs.hopperADetected) count++;
        if (inputs.hopperBDetected) count++;
        return count;
    }

    /**
     * Returns true if the hopper is full (both positions have game pieces).
     */
    public boolean isHopperFull() {
        return inputs.hopperADetected && inputs.hopperBDetected;
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
    public double getConveyorCurrentAmps() {
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

    // ========== Jam Detection Methods ==========

    /**
     * Checks if the hopper (conveyor) motor is jammed.
     * A jam is detected when current is high but velocity is low.
     *
     * @return true if the hopper conveyor motor is jammed
     */
    public boolean isHopperJammed() {
        return (inputs.conveyorCurrentAmps >= Constants.Indexer.HOPPER_JAM_CURRENT_THRESHOLD)
            && (Math.abs(inputs.conveyorVelocityRPS) <= Constants.Indexer.HOPPER_JAM_VELOCITY_THRESHOLD);
    }

    /**
     * Checks if the indexer motor is jammed.
     * A jam is detected when current is high but velocity is low.
     *
     * @return true if the indexer motor is jammed
     */
    public boolean isIndexerJammed() {
        return (inputs.indexerCurrentAmps >= Constants.Indexer.INDEXER_JAM_CURRENT_THRESHOLD)
            && (Math.abs(inputs.indexerVelocityRPS) <= Constants.Indexer.INDEXER_JAM_VELOCITY_THRESHOLD);
    }
}