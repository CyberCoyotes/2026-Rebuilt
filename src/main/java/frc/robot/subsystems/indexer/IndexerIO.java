package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * IndexerIO - Hardware abstraction interface for the indexer subsystem.
 *
 * The indexer moves game pieces from the intake through the robot to the shooter.
 * It consists of:
 * - Conveyor motor: Moves pieces along the conveyor of the hopper toward the indexer
 * - Indexer motor: Grabs pieces from the hopper and feeds them into the shooter
 * - Indexer ToF sensor: Detects when a game piece is staged and ready to shoot
 * - Hopper ToF sensors (A, B, C): Detect game pieces at different positions in hopper
 *
 * PATTERN: IO Interface
 * - IndexerIO: Interface defining what indexer hardware can do
 * - IndexerIOHardware: Real hardware implementation with motors and sensors
 * - IndexerIOSim: Simulation for testing without hardware
 *
 * @see Constants.Indexer for hardware configuration
 * @see IndexerSubsystemBasic for a simpler direct-hardware approach (good for learning)
 */
public interface IndexerIO {

    /**
     * IndexerIOInputs - Container for all indexer sensor data.
     *
     * This class holds all data we read from the indexer hardware each cycle.
     * Implements LoggableInputs for automatic AdvantageKit logging and replay.
     */
    class IndexerIOInputs implements LoggableInputs {
        // ===== Conveyor Motor Data =====
        /** Conveyor motor velocity in rotations per second */
        public double conveyorVelocityRPS = 0.0;

        /** Conveyor motor applied voltage */
        public double conveyorAppliedVolts = 0.0;

        /** Conveyor motor supply current in amps */
        public double conveyorCurrentAmps = 0.0;

        /** Conveyor motor temperature in Celsius */
        public double conveyorTempCelsius = 0.0;

        // ===== Indexer Motor Data =====
        /** Indexer motor velocity in rotations per second */
        public double indexerVelocityRPS = 0.0;

        /** Indexer motor applied voltage */
        public double indexerAppliedVolts = 0.0;

        /** Indexer motor supply current in amps */
        public double indexerCurrentAmps = 0.0;

        /** Indexer motor temperature in Celsius */
        public double indexerTempCelsius = 0.0;

        // ===== Indexer Time-of-Flight Sensor Data =====
        /** Distance reading from indexer ToF sensor in millimeters */
        public double tofDistanceMM = 0.0;

        /** True if indexer ToF sensor has valid measurement */
        public boolean tofValid = false;

        /** True if a game piece is detected at the indexer (ready to shoot) */
        public boolean gamePieceDetected = false;

        // ===== Hopper Time-of-Flight Sensor Data =====
        /** Distance reading from hopper A ToF sensor in millimeters */
        public double hopperADistanceMM = 0.0;

        /** True if hopper A ToF sensor has valid measurement */
        public boolean hopperAValid = false;

        /** True if a game piece is detected at hopper position A */
        public boolean hopperADetected = false;

        /** Distance reading from hopper B ToF sensor in millimeters */
        public double hopperBDistanceMM = 0.0;

        /** True if hopper B ToF sensor has valid measurement */
        public boolean hopperBValid = false;

        /** True if a game piece is detected at hopper position B */
        public boolean hopperBDetected = false;

        /** Distance reading from hopper C ToF sensor in millimeters */
        public double hopperCDistanceMM = 0.0;

        /** True if hopper C ToF sensor has valid measurement */
        public boolean hopperCValid = false;

        /** True if a game piece is detected at hopper position C */
        public boolean hopperCDetected = false;

        @Override
        public void toLog(LogTable table) {
            // Conveyor motor
            table.put("ConveyorVelocityRPS", conveyorVelocityRPS);
            table.put("ConveyorAppliedVolts", conveyorAppliedVolts);
            table.put("ConveyorCurrentAmps", conveyorCurrentAmps);
            table.put("ConveyorTempCelsius", conveyorTempCelsius);

            // Indexer motor
            table.put("IndexerVelocityRPS", indexerVelocityRPS);
            table.put("IndexerAppliedVolts", indexerAppliedVolts);
            table.put("IndexerCurrentAmps", indexerCurrentAmps);
            table.put("IndexerTempCelsius", indexerTempCelsius);

            // Indexer ToF sensor
            table.put("TofDistanceMM", tofDistanceMM);
            table.put("TofValid", tofValid);
            table.put("GamePieceDetected", gamePieceDetected);

            // Hopper ToF sensors
            table.put("HopperADistanceMM", hopperADistanceMM);
            table.put("HopperAValid", hopperAValid);
            table.put("HopperADetected", hopperADetected);

            table.put("HopperBDistanceMM", hopperBDistanceMM);
            table.put("HopperBValid", hopperBValid);
            table.put("HopperBDetected", hopperBDetected);

            table.put("HopperCDistanceMM", hopperCDistanceMM);
            table.put("HopperCValid", hopperCValid);
            table.put("HopperCDetected", hopperCDetected);
        }

        @Override
        public void fromLog(LogTable table) {
            // Conveyor motor
            conveyorVelocityRPS = table.get("ConveyorVelocityRPS", conveyorVelocityRPS);
            conveyorAppliedVolts = table.get("ConveyorAppliedVolts", conveyorAppliedVolts);
            conveyorCurrentAmps = table.get("ConveyorCurrentAmps", conveyorCurrentAmps);
            conveyorTempCelsius = table.get("ConveyorTempCelsius", conveyorTempCelsius);

            // Indexer motor
            indexerVelocityRPS = table.get("IndexerVelocityRPS", indexerVelocityRPS);
            indexerAppliedVolts = table.get("IndexerAppliedVolts", indexerAppliedVolts);
            indexerCurrentAmps = table.get("IndexerCurrentAmps", indexerCurrentAmps);
            indexerTempCelsius = table.get("IndexerTempCelsius", indexerTempCelsius);

            // Indexer ToF sensor
            tofDistanceMM = table.get("TofDistanceMM", tofDistanceMM);
            tofValid = table.get("TofValid", tofValid);
            gamePieceDetected = table.get("GamePieceDetected", gamePieceDetected);

            // Hopper ToF sensors
            hopperADistanceMM = table.get("HopperADistanceMM", hopperADistanceMM);
            hopperAValid = table.get("HopperAValid", hopperAValid);
            hopperADetected = table.get("HopperADetected", hopperADetected);

            hopperBDistanceMM = table.get("HopperBDistanceMM", hopperBDistanceMM);
            hopperBValid = table.get("HopperBValid", hopperBValid);
            hopperBDetected = table.get("HopperBDetected", hopperBDetected);

            hopperCDistanceMM = table.get("HopperCDistanceMM", hopperCDistanceMM);
            hopperCValid = table.get("HopperCValid", hopperCValid);
            hopperCDetected = table.get("HopperCDetected", hopperCDetected);
        }
    }

    /**
     * Updates inputs from hardware.
     * Called periodically (every 20ms) by IndexerSubsystem.
     *
     * @param inputs The IndexerIOInputs object to populate with current data
     */
    default void updateInputs(IndexerIOInputs inputs) {}

    /**
     * Sets the conveyor motor speed as a percentage.
     *
     * @param percent Motor speed from -1.0 to 1.0 (+ = toward indexer, - = reverse)
     */
    default void setConveyorMotor(double percent) {}

    /**
     * Sets the indexer motor speed as a percentage.
     *
     * @param percent Motor speed from -1.0 to 1.0 (+ = toward shooter, - = reverse)
     */
    default void setIndexerMotor(double percent) {}

    /**
     * Stops both motors.
     */
    default void stop() {}
}
