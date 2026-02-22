package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/**
 * IndexerIO - Hardware abstraction interface for the indexer subsystem.
 *
 * The indexer moves game pieces from the intake to the shooter via:
 * - Conveyor motor (TalonFXS/Minion): Moves pieces along the hopper
 * - Indexer motor (TalonFX/Kraken): Feeds pieces into the shooter
 * - Hopper ToF A (CANrange): Detects game pieces near the front of the hopper
 * - Hopper ToF B (CANrange): Detects game pieces near the back of the hopper
 *
 * PATTERN: IO Interface
 * - IndexerIO:          Interface defining what the indexer hardware can do (this file)
 * - IndexerIOHardware:  Real hardware implementation
 * - IndexerIOSim:       Simulation implementation for testing without hardware
 *
 * @see Constants.Indexer for CAN IDs and hardware configuration
 * @see IndexerSubsystem for the subsystem that uses this interface
 */
public interface IndexerIO {

    /**
     * IndexerIOInputs — All sensor data read from indexer hardware each cycle.
     *
     * The @AutoLog annotation generates IndexerIOInputsAutoLogged, a subclass
     * that handles AdvantageKit logging and replay. The subsystem must instantiate
     * IndexerIOInputsAutoLogged (not IndexerIOInputs directly) for logging to work.
     *
     * Distance fields (hopperA/BDistanceMeters) are included for threshold tuning —
     * they let you watch raw sensor output in AdvantageScope while adjusting
     * TOF_DETECTION_THRESHOLD_METERS in IndexerIOHardware.
     */
    @AutoLog
    class IndexerIOInputs {

        // ===== Conveyor Motor =====
        /** Conveyor motor velocity in rotations per second. Used for jam detection. */
        public double conveyorVelocityRPS = 0.0;

        /** Conveyor motor supply current in amps. Used for jam detection. */
        public double conveyorCurrentAmps = 0.0;

        // ===== Indexer Motor =====
        /** Indexer motor velocity in rotations per second. Used for jam detection. */
        public double indexerVelocityRPS = 0.0;

        /** Indexer motor supply current in amps. Used for jam detection. */
        public double indexerCurrentAmps = 0.0;

        // ===== Hopper CANrange A =====
        /**
         * Raw distance from CANrange A in meters.
         * Watch this in AdvantageScope when tuning TOF_DETECTION_THRESHOLD_METERS.
         */
        public double hopperADistanceMeters = 0.0;

        /** True if a game piece is detected at hopper position A. */
        public boolean hopperADetected = false;

        // ===== Hopper CANrange B =====
        /**
         * Raw distance from CANrange B in meters.
         * Watch this in AdvantageScope when tuning TOF_DETECTION_THRESHOLD_METERS.
         */
        public double hopperBDistanceMeters = 0.0;

        /** True if a game piece is detected at hopper position B. */
        public boolean hopperBDetected = false;
    }

    /**
     * Updates inputs with current sensor data.
     * Called every 20ms by IndexerSubsystem.periodic().
     *
     * @param inputs The IndexerIOInputsAutoLogged object to populate
     */
    default void updateInputs(IndexerIOInputs inputs) {}

    /**
     * Sets the conveyor motor output voltage.
     *
     * @param volts Positive = toward indexer, negative = reverse
     */
    default void setConveyorMotor(double volts) {}

    /**
     * Sets the indexer motor output voltage.
     *
     * @param volts Positive = toward shooter, negative = reverse
     */
    default void setIndexerMotor(double volts) {}

    /** Stops both motors. */
    default void stop() {}
}