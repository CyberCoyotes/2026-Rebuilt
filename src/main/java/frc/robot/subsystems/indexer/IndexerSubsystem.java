package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

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
    private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();

    /**
     * Creates a new IndexerSubsystem with the specified IO implementation.
     *
     * @param io The IndexerIO implementation (hardware or simulation)
     */
    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Update inputs from hardware/simulation every cycle (20ms)
        io.updateInputs(inputs);

        // Log all inputs for AdvantageKit replay
        Logger.processInputs("Indexer", inputs);
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
