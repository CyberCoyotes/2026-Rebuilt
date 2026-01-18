package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.util.Units;

/**
 * VisionIOLimelight - Limelight hardware implementation of VisionIO interface.
 *
 * This class handles all communication with the Limelight camera via NetworkTables.
 * It reads vision data and provides it to the VisionSubsystem in a hardware-independent format.
 *
 * Key Features:
 * - Caches NetworkTable entries for performance
 * - Accurately calculates timestamps accounting for latency
 * - Converts angles to radians for consistency
 * - Thread-safe synchronized methods
 * - Defensive copying of arrays
 *
 * NetworkTables Reference (Limelight):
 * - tv: Valid target (0 or 1)
 * - tx: Horizontal offset to target in degrees
 * - ty: Vertical offset to target in degrees
 * - ta: Target area (0-100% of image)
 * - tid: AprilTag ID
 * - botpose: Robot pose from AprilTag [x, y, z, roll, pitch, yaw]
 * - tl: Pipeline latency (ms)
 * - cl: Capture latency (ms)
 */
public class VisionIOLimelight implements VisionIO {

    // ===== NetworkTable Entries =====
    private final NetworkTable limelightTable;
    private final NetworkTableEntry validEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry botposeEntry;
    private final NetworkTableEntry pipelineLatencyEntry;
    private final NetworkTableEntry captureLatencyEntry;
    private final NetworkTableEntry ledModeEntry;
    private final NetworkTableEntry pipelineEntry;

    /**
     * Creates a new VisionIOLimelight instance.
     *
     * @param limelightName The NetworkTable name of the Limelight (e.g., "limelight", "limelight-front")
     */
    public VisionIOLimelight(String limelightName) {
        // Get the Limelight's NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

        // Cache NetworkTable entries for performance (avoid repeated lookups)
        validEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        taEntry = limelightTable.getEntry("ta");
        tagIdEntry = limelightTable.getEntry("tid");
        botposeEntry = limelightTable.getEntry("botpose");
        pipelineLatencyEntry = limelightTable.getEntry("tl");
        captureLatencyEntry = limelightTable.getEntry("cl");
        ledModeEntry = limelightTable.getEntry("ledMode");
        pipelineEntry = limelightTable.getEntry("pipeline");

        // Initialize Limelight to known state
        setLEDMode(LEDMode.PIPELINE_DEFAULT);
        setPipeline(0); // Default to pipeline 0 (AprilTags)
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        // Read latency values first
        double pipelineLatency = pipelineLatencyEntry.getDouble(0.0);
        double captureLatency = captureLatencyEntry.getDouble(0.0);
        double totalLatency = pipelineLatency + captureLatency;

        // Calculate accurate timestamp by subtracting total latency
        // This gives us when the image was actually captured, not when we read it
        inputs.timestamp = Timer.getFPGATimestamp() - (totalLatency / 1000.0);

        // Store latency values
        inputs.pipelineLatencyMs = pipelineLatency;
        inputs.captureLatencyMs = captureLatency;
        inputs.totalLatencyMs = totalLatency;

        // Check if we have a valid target
        inputs.hasTargets = validEntry.getDouble(0.0) > 0.5; // Use 0.5 threshold for robustness

        if (inputs.hasTargets) {
            // Read target data and convert to standard units (radians)
            inputs.horizontalAngleRadians = Units.degreesToRadians(txEntry.getDouble(0.0));
            inputs.verticalAngleRadians = Units.degreesToRadians(tyEntry.getDouble(0.0));
            inputs.targetArea = taEntry.getDouble(0.0);

            // Read AprilTag specific data
            inputs.tagId = (int) tagIdEntry.getDouble(-1.0);

            // Clone botpose array for defensive copy (prevents external modification)
            double[] rawBotpose = botposeEntry.getDoubleArray(new double[6]);
            inputs.botpose = rawBotpose.clone();
        } else {
            // No target detected - reset all values to zero/invalid
            inputs.horizontalAngleRadians = 0.0;
            inputs.verticalAngleRadians = 0.0;
            inputs.targetArea = 0.0;
            inputs.tagId = -1;
            inputs.botpose = new double[6]; // All zeros
        }
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        pipelineEntry.setNumber(pipelineIndex);
    }

    @Override
    public void setLEDMode(LEDMode mode) {
        ledModeEntry.setNumber(mode.value);
    }
}
