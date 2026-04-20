package frc.robot.subsystems.indexer;

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
     * Distance fields (hopperA/BDistanceMeters) are included for threshold tuning —
     * they let you watch raw sensor output in AdvantageScope while adjusting
     * TOF_DETECTION_THRESHOLD_METERS in IndexerIOHardware.
     */
    class IndexerIOInputs {

        // ===== Conveyor Motor =====
        /** Conveyor motor velocity in rotations per second. Used for jam detection. */
        public double conveyorVelocityRPS = 0.0;

        /** Conveyor motor supply current in amps. Used for jam detection. */
        public double conveyorCurrentAmps = 0.0;

        // ===== Kicker Motor =====
        /** Kicker motor velocity in rotations per second. Used for jam detection. */
        public double kickerLeadVelocityRPS = 0.0;

        /** Kicker motor supply current in amps. Used for jam detection. */
        public double kickerLeadCurrentAmps = 0.0;
        public double kickerFollowCurrentAmps = 0.0;

        // ===== Chute CANrange =====
        /**
         * Raw distance from Chute CANrange in meters.
         * Watch this in AdvantageScope when tuning CHUTE_MAX_DISTANCE or the sensor's
         * ProximityThreshold/Hysteresis in IndexerIOHardware.
         */
        public double chuteDistanceMeters = 0.0;

        /** True if a game piece is detected in the indexer→shooter chute. */
        public boolean chuteDetected = false;

    }

    /**
     * Updates inputs with current sensor data.
     * Called every 20ms by IndexerSubsystem.periodic().
     *
     * @param inputs The IndexerIOInputs object to populate
     */
    default void updateInputs(IndexerIOInputs inputs) {}

    /**
     * Sets the conveyor motor to a closed-loop velocity target (VelocityVoltage, Slot 0).
     * Use for normal forward operation.
     *
     * @param rps Target velocity in rotations per second (positive = toward indexer)
     */
    default void setConveyorVelocity(double rps) {}

    /**
     * Sets the conveyor motor output voltage (VoltageOut).
     * Used for reverse and popper; kept as fallback for forward if needed.
     *
     * @param volts Positive = toward indexer, negative = reverse
     */
    default void setConveyorMotor(double volts) {}

    /**
     * Sets the kicker motor output voltage.
     *
     * @param volts Positive = toward shooter, negative = reverse
     */
    default void setKickerMotorVolts(double volts) {}

    /** Stops both motors. */
    default void stop() {}
}
