package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * IndexerIO - Hardware abstraction interface for the indexer subsystem.
 *
 * The indexer moves game pieces from the intake through the robot to the shooter.
 * It consists of:
 * - Floor motor: Moves pieces along the floor of the hopper toward the feeder
 * - Feeder motor: Grabs pieces from the hopper and feeds them into the shooter
 * - Feeder ToF sensor: Detects when a game piece is staged and ready to shoot
 * - Hopper ToF sensors (A, B, C): Detect game pieces at different positions in hopper
 *
 * HARDWARE:
 * - 2x Kraken X60 motors (floor + feeder)
 * - 4x Playing With Fusion Time-of-Flight sensors (1 feeder + 3 hopper)
 *
 * PATTERN: IO Interface
 * - IndexerIO: Interface defining what indexer hardware can do
 * - IndexerIOHardware: Real hardware implementation with motors and sensors
 * - IndexerIOSim: Simulation for testing without hardware
 *
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
        // ===== Floor Motor Data =====
        /** Floor motor velocity in rotations per second */
        public double floorVelocityRPS = 0.0;

        /** Floor motor applied voltage */
        public double floorAppliedVolts = 0.0;

        /** Floor motor supply current in amps */
        public double floorCurrentAmps = 0.0;

        /** Floor motor temperature in Celsius */
        public double floorTempCelsius = 0.0;

        // ===== Feeder Motor Data =====
        /** Feeder motor velocity in rotations per second */
        public double feederVelocityRPS = 0.0;

        /** Feeder motor applied voltage */
        public double feederAppliedVolts = 0.0;

        /** Feeder motor supply current in amps */
        public double feederCurrentAmps = 0.0;

        /** Feeder motor temperature in Celsius */
        public double feederTempCelsius = 0.0;

        // ===== Feeder Time-of-Flight Sensor Data =====
        /** Distance reading from feeder ToF sensor in millimeters */
        public double tofDistanceMM = 0.0;

        /** True if feeder ToF sensor has valid measurement */
        public boolean tofValid = false;

        /** True if a game piece is detected at the feeder (ready to shoot) */
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
            // Floor motor
            table.put("FloorVelocityRPS", floorVelocityRPS);
            table.put("FloorAppliedVolts", floorAppliedVolts);
            table.put("FloorCurrentAmps", floorCurrentAmps);
            table.put("FloorTempCelsius", floorTempCelsius);

            // Feeder motor
            table.put("FeederVelocityRPS", feederVelocityRPS);
            table.put("FeederAppliedVolts", feederAppliedVolts);
            table.put("FeederCurrentAmps", feederCurrentAmps);
            table.put("FeederTempCelsius", feederTempCelsius);

            // Feeder ToF sensor
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
            // Floor motor
            floorVelocityRPS = table.get("FloorVelocityRPS", floorVelocityRPS);
            floorAppliedVolts = table.get("FloorAppliedVolts", floorAppliedVolts);
            floorCurrentAmps = table.get("FloorCurrentAmps", floorCurrentAmps);
            floorTempCelsius = table.get("FloorTempCelsius", floorTempCelsius);

            // Feeder motor
            feederVelocityRPS = table.get("FeederVelocityRPS", feederVelocityRPS);
            feederAppliedVolts = table.get("FeederAppliedVolts", feederAppliedVolts);
            feederCurrentAmps = table.get("FeederCurrentAmps", feederCurrentAmps);
            feederTempCelsius = table.get("FeederTempCelsius", feederTempCelsius);

            // Feeder ToF sensor
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
     * Sets the floor motor speed as a percentage.
     *
     * @param percent Motor speed from -1.0 to 1.0 (+ = toward feeder, - = reverse)
     */
    default void setFloorMotor(double percent) {}

    /**
     * Sets the feeder motor speed as a percentage.
     *
     * @param percent Motor speed from -1.0 to 1.0 (+ = toward shooter, - = reverse)
     */
    default void setFeederMotor(double percent) {}

    /**
     * Stops both motors.
     */
    default void stop() {}
}
