package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/**
 * IndexerIO - Hardware abstraction interface for the indexer subsystem.
 *
 * The indexer moves game pieces from the intake through the robot to the shooter.
 * It consists of:
 * - Conveyor motor: Moves pieces along the conveyor of the hopper toward the indexer
 * - Indexer motor: Grabs pieces from the hopper and feeds them into the shooter
 *
 * PATTERN: IO Interface
 * - IndexerIO: Interface defining what indexer hardware can do
 * - IndexerIOHardware: Real hardware implementation with motors and sensors
 * - IndexerIOSim: Simulation for testing without hardware
 *
 * @see Constants.Indexer for hardware configuration
 */
public interface IndexerIO {

    /**
     * IndexerIOInputs - Container for all indexer sensor data.
     *
     * Fields are split into fast (updated every 20ms) and slow (updated at 10Hz).
     * Fast fields drive control; slow fields are for diagnostics and dashboard only.
     */
    @AutoLog
    class IndexerIOInputs {

        // ===== Fast Fields (updated every 20ms cycle) =====

        /** Conveyor motor velocity in rotations per second */
        public double conveyorVelocityRPS = 0.0;

        /** Conveyor motor applied voltage */
        public double conveyorAppliedVolts = 0.0;

        /** Indexer motor velocity in rotations per second */
        public double indexerVelocityRPS = 0.0;

        /** Indexer motor applied voltage */
        public double indexerAppliedVolts = 0.0;

        // ===== Slow Fields (updated at 10Hz — diagnostics only) =====

        /** Conveyor motor supply current in amps */
        public double conveyorCurrentAmps = 0.0;

        /** Indexer motor supply current in amps */
        public double indexerCurrentAmps = 0.0;

        // ===== Hopper Time-of-Flight Sensors (not currently wired) =====
        // Uncomment when CANrange sensors are installed and configured.

        // public double hopperADistanceMM = 0.0;
        // public boolean hopperAValid = false;
        // public boolean hopperADetected = false;

        // public double hopperBDistanceMM = 0.0;
        // public boolean hopperBValid = false;
        // public boolean hopperBDetected = false;

        // public double hopperCDistanceMM = 0.0;
        // public boolean hopperCValid = false;
        // public boolean hopperCDetected = false;
    }

    /**
     * Updates control-critical inputs from hardware.
     * Called every cycle (every 20ms) by IndexerSubsystem.
     * Only refreshes fast signals: velocity and applied voltage for both motors.
     *
     * @param inputs The IndexerIOInputs object to populate with current data
     */
    default void updateInputs(IndexerIOInputs inputs) {}

    /**
     * Updates diagnostic inputs from hardware.
     * Called at 10Hz (every 5th cycle) by IndexerSubsystem.
     * Refreshes slow signals: supply current for both motors.
     *
     * WHY SEPARATE? Current signals are set to 10Hz on the CAN bus.
     * Refreshing them every 20ms just reads stale cached data and wastes JNI calls.
     *
     * @param inputs The IndexerIOInputs object to populate with diagnostic data
     */
    default void updateSlowInputs(IndexerIOInputs inputs) {}

    /**
     * Sets the conveyor motor output voltage.
     *
     * @param volts Motor voltage (+ = toward indexer, - = reverse)
     */
    default void setConveyorMotor(double volts) {}

    /**
     * Sets the indexer motor output voltage.
     *
     * @param volts Motor voltage (+ = toward shooter, - = reverse)
     */
    default void setIndexerMotor(double volts) {}

    /**
     * Stops both motors.
     */
    default void stop() {}
}