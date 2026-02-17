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
import frc.robot.Robot;
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

    private final IndexerIOInputs inputs = new IndexerIOInputs();

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

    // ===== Jam Detection Telemetry Publishers =====

    // FIXME These don't belong here

    // These publish jam status to the Elastic dashboard so drivers can see if something is stuck
    private final BooleanPublisher hopperJammedPublisher; 
    private final BooleanPublisher indexerJammedPublisher;
    private final DoublePublisher conveyorCurrentPublisher;
    private final DoublePublisher indexerCurrentPublisher;


        // ===== Jam Detection Thresholds =====
    // Pattern: A motor is jammed when current is HIGH but velocity is LOW.
    // This means the motor is trying to spin but something is blocking it.

    /** Conveyor (hopper) motor jam current threshold in amps (TODO: Tune on robot) */
    public static final double HOPPER_JAM_CURRENT_THRESHOLD = 20.0;

    /** Conveyor (hopper) motor jam velocity threshold in RPS (TODO: Tune on robot) */
    public static final double HOPPER_JAM_VELOCITY_THRESHOLD = 0.5;

    /** Indexer motor jam current threshold in amps (TODO: Tune on robot) */
    public static final double INDEXER_JAM_CURRENT_THRESHOLD = 20.0;

    /** Indexer motor jam velocity threshold in RPS (TODO: Tune on robot) */
    public static final double INDEXER_JAM_VELOCITY_THRESHOLD = 0.5;

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

        // Initialize jam detection publishers for dashboard visibility
        hopperJammedPublisher = indexerTable.getBooleanTopic("HopperJammed").publish();
        indexerJammedPublisher = indexerTable.getBooleanTopic("IndexerJammed").publish();
        conveyorCurrentPublisher = indexerTable.getDoubleTopic("ConveyorCurrentAmps").publish();
        indexerCurrentPublisher = indexerTable.getDoubleTopic("IndexerCurrentAmps").publish();
    }

    @Override
    public void periodic() {
        // Update inputs from hardware/simulation every cycle (20ms)
        io.updateInputs(inputs);

        // Log all inputs for AdvantageKit replay
        // if (Robot.ENABLE_ADVANTAGEKIT) {  // make it public or put in Constants
        //     Logger.processInputs("Indexer", inputs);
        // }

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

        // Publish jam detection status and current draw for dashboard alerts
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


    // Convenience methods for common actions
    public void indexerForward() {
        io.setIndexerMotor(INDEXER_FORWARD_VOLTAGE); // TODO: Tune indexer voltage on robot
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

    // ========== Jam Detection Methods ==========
    // Jam detection uses a dual-threshold approach:
    //   - HIGH current means the motor is trying hard to spin
    //   - LOW velocity means the motor is barely moving (or stalled)
    //   - Both conditions together indicate something is physically blocking the motor
    //
    // This pattern is consistent with IntakeSubsystem.isJammed() for team familiarity.
    // Thresholds are defined in Constants.Indexer and should be tuned on the real robot.

    /**
     * Checks if the hopper (conveyor) motor is jammed.
     *
     * A jam is detected when the conveyor motor is drawing high current
     * but spinning at low velocity. This means a game piece (or multiple
     * pieces) is stuck and the motor can't push past it.
     *
     * Common hopper jam causes:
     * - Game pieces wedged against each other in the conveyor
     * - Too many pieces fed in at once from intake
     * - A piece caught between the conveyor and the hopper walls
     *
     * @return true if the hopper conveyor motor is jammed
     * @see Constants.Indexer#HOPPER_JAM_CURRENT_THRESHOLD
     * @see Constants.Indexer#HOPPER_JAM_VELOCITY_THRESHOLD
     */
    public boolean isHopperJammed() {
        // Read current draw - high current means the motor is working hard
        double current = inputs.conveyorCurrentAmps;

        // Read velocity - low velocity means the motor is not spinning freely
        double velocity = inputs.conveyorVelocityRPS;

        // Jam = high current AND low velocity (motor stalled under load)
        return (current >= HOPPER_JAM_CURRENT_THRESHOLD)
            && (Math.abs(velocity) <= HOPPER_JAM_VELOCITY_THRESHOLD);
    }

    /**
     * Checks if the indexer motor is jammed.
     *
     * A jam is detected when the indexer motor is drawing high current
     * but spinning at low velocity. This means a game piece is stuck
     * at the indexer and can't be fed to the shooter.
     *
     * Common indexer jam causes:
     * - Game piece stuck between the indexer wheel and the shooter entrance
     * - Misaligned game piece unable to pass into the shooter
     * - Multiple pieces competing to enter the shooter at once
     *
     * @return true if the indexer motor is jammed
     * @see Constants.Indexer#INDEXER_JAM_CURRENT_THRESHOLD
     * @see Constants.Indexer#INDEXER_JAM_VELOCITY_THRESHOLD
     */
    public boolean isIndexerJammed() {
        // Read current draw - high current means the motor is working hard
        double current = inputs.indexerCurrentAmps;

        // Read velocity - low velocity means the motor is not spinning freely
        double velocity = inputs.indexerVelocityRPS;

        // Jam = high current AND low velocity (motor stalled under load)
        return (current >= INDEXER_JAM_CURRENT_THRESHOLD)
            && (Math.abs(velocity) <= INDEXER_JAM_VELOCITY_THRESHOLD);
    }
}
