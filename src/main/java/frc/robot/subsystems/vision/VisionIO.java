package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * VisionIO - Hardware abstraction interface for vision systems (Limelight, PhotonVision, etc.)
 *
 * This interface defines the contract for vision hardware implementations, allowing the
 * VisionSubsystem to work with different camera systems without code changes.
 *
 * Pattern: IO Interface (inspired by FRC 2910, 254, 1678)
 * - VisionIO: Interface defining what vision hardware can do
 * - VisionIOLimelight: Implementation for Limelight cameras
 * - VisionIOSim: Implementation for simulation/testing
 *
 * Benefits:
 * - Hardware independence: Swap cameras without changing subsystem code
 * - Testability: Mock vision data for testing without hardware
 * - AdvantageKit integration: Clean logging and replay
 */
public interface VisionIO {

    /**
     * Updates the vision inputs with the latest data from the camera.
     * Called periodically (typically every 20ms) by VisionSubsystem.
     *
     * @param inputs The VisionIOInputs object to populate with current data
     */
    default void updateInputs(VisionIOInputs inputs) {}

    /**
     * Sets the active vision pipeline on the camera.
     * Different pipelines can be configured for different targets (AprilTags, game pieces, etc.)
     *
     * @param pipelineIndex The pipeline index to activate (0-9 for Limelight)
     */
    default void setPipeline(int pipelineIndex) {}

    /**
     * Controls the camera's LED mode.
     *
     * @param mode The LED mode to set
     */
    default void setLEDMode(LEDMode mode) {}

    /**
     * LED control modes for vision cameras.
     */
    enum LEDMode {
        /** Use the LED mode specified by the current pipeline */
        PIPELINE_DEFAULT(0),

        /** Force LEDs off */
        FORCE_OFF(1),

        /** Force LEDs to blink */
        FORCE_BLINK(2),

        /** Force LEDs on */
        FORCE_ON(3);

        public final int value;

        LEDMode(int value) {
            this.value = value;
        }
    }

    /**
     * VisionIOInputs - Container for all vision data read from hardware.
     *
     * This class holds all the data we read from the camera each cycle.
     * It implements LoggableInputs for automatic AdvantageKit logging.
     *
     * All fields are public for easy access by VisionSubsystem.
     */
    class VisionIOInputs implements LoggableInputs {
        // ===== Timestamp Data =====
        /**
         * Timestamp when this vision data was captured (FPGA time in seconds).
         * Accounts for pipeline and capture latency for accurate pose estimation.
         */
        public double timestamp = 0.0;

        // ===== Target Detection =====
        /** True if the camera sees a valid target (tv = 1) */
        public boolean hasTargets = false;

        /** Horizontal angle to target in radians (tx converted from degrees) */
        public double horizontalAngleRadians = 0.0;

        /** Vertical angle to target in radians (ty converted from degrees) */
        public double verticalAngleRadians = 0.0;

        /** Target area as percentage of image (0.0 to 100.0) */
        public double targetArea = 0.0;

        // ===== AprilTag Specific Data =====
        /** AprilTag fiducial ID (-1 if no tag detected) */
        public int tagId = -1;

        /**
         * Robot pose in field coordinates from AprilTag (botpose).
         * Array: [x, y, z, roll, pitch, yaw] in meters and degrees.
         */
        public double[] botpose = new double[6];

        // ===== Latency Data =====
        /** Pipeline processing latency in milliseconds */
        public double pipelineLatencyMs = 0.0;

        /** Image capture latency in milliseconds */
        public double captureLatencyMs = 0.0;

        /** Total latency (pipeline + capture) in milliseconds */
        public double totalLatencyMs = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("Timestamp", timestamp);
            table.put("HasTargets", hasTargets);
            table.put("HorizontalAngleRad", horizontalAngleRadians);
            table.put("VerticalAngleRad", verticalAngleRadians);
            table.put("TargetArea", targetArea);
            table.put("TagId", tagId);
            table.put("Botpose", botpose);
            table.put("PipelineLatencyMs", pipelineLatencyMs);
            table.put("CaptureLatencyMs", captureLatencyMs);
            table.put("TotalLatencyMs", totalLatencyMs);
        }

        @Override
        public void fromLog(LogTable table) {
            timestamp = table.get("Timestamp", timestamp);
            hasTargets = table.get("HasTargets", hasTargets);
            horizontalAngleRadians = table.get("HorizontalAngleRad", horizontalAngleRadians);
            verticalAngleRadians = table.get("VerticalAngleRad", verticalAngleRadians);
            targetArea = table.get("TargetArea", targetArea);
            tagId = table.get("TagId", tagId);
            botpose = table.get("Botpose", botpose);
            pipelineLatencyMs = table.get("PipelineLatencyMs", pipelineLatencyMs);
            captureLatencyMs = table.get("CaptureLatencyMs", captureLatencyMs);
            totalLatencyMs = table.get("TotalLatencyMs", totalLatencyMs);
        }
    }
}
