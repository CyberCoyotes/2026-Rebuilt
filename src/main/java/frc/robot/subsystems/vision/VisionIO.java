package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * VisionIO - Hardware abstraction interface for vision systems (Limelight, etc.)
 *
 * Implementations read from camera hardware and populate VisionIOInputs.
 * VisionSubsystem consumes this via updateInputs() each periodic cycle.
 *
 * Pose data flows:
 *   MegaTag2 (primary)  — IMU-seeded 3D pose, fed to drivetrain addVisionMeasurement()
 *   MegaTag1 (fallback) — 2D pose, fed when MegaTag2 has no tags
 *   TX (raw)            — Limelight horizontal offset in degrees, used for rotation correction
 */
public interface VisionIO {

    /**
     * Reads all vision data into inputs.
     *
     * @param inputs          Container to populate with current camera data
     * @param robotYawDegrees Current robot heading from IMU — required for MegaTag2
     */
    default void updateInputs(VisionIOInputs inputs, double robotYawDegrees) {}

    default void setPipeline(int pipelineIndex) {}

    default void setLEDMode(LEDMode mode) {}

    enum LEDMode {
        PIPELINE_DEFAULT(0),
        FORCE_OFF(1),
        FORCE_BLINK(2),
        FORCE_ON(3);

        public final int value;

        LEDMode(int value) {
            this.value = value;
        }
    }

    /**
     * All camera data for one periodic cycle.
     *
     * Pose arrays are [x_meters, y_meters, yaw_degrees] in WPILib blue-origin field coordinates.
     * Both maps must be updated together — they are co-indexed on timestamp.
     */
    class VisionIOInputs implements LoggableInputs {

        // ── MegaTag2 (primary — IMU-seeded, rotation-locked) ──────────────────
        public boolean megaTag2Valid    = false;
        public double[] megaTag2Pose    = new double[3]; // [x_m, y_m, yaw_deg]
        public double megaTag2Timestamp = 0.0;
        public int    megaTag2TagCount  = 0;
        public double megaTag2AvgTagDistM = 0.0;

        // ── MegaTag1 (fallback — 2D, no IMU required) ─────────────────────────
        public boolean megaTag1Valid    = false;
        public double[] megaTag1Pose    = new double[3]; // [x_m, y_m, yaw_deg]
        public double megaTag1Timestamp = 0.0;
        public int    megaTag1TagCount  = 0;

        // ── Raw target data (for rotation control and target validation) ───────
        public boolean hasTargets  = false;
        public double  txDegrees   = 0.0;  // horizontal offset to primary tag, + = right
        public double  tyDegrees   = 0.0;  // vertical offset to primary tag,   + = up
        public double  targetArea  = 0.0;  // % of image
        public int     tagId       = -1;   // primary AprilTag ID, -1 = none
        public double  totalLatencyMs = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("MegaTag2/Valid",        megaTag2Valid);
            table.put("MegaTag2/Pose",         megaTag2Pose);
            table.put("MegaTag2/Timestamp",    megaTag2Timestamp);
            table.put("MegaTag2/TagCount",     megaTag2TagCount);
            table.put("MegaTag2/AvgTagDistM",  megaTag2AvgTagDistM);

            table.put("MegaTag1/Valid",        megaTag1Valid);
            table.put("MegaTag1/Pose",         megaTag1Pose);
            table.put("MegaTag1/Timestamp",    megaTag1Timestamp);
            table.put("MegaTag1/TagCount",     megaTag1TagCount);

            table.put("HasTargets",   hasTargets);
            table.put("TX_deg",       txDegrees);
            table.put("TY_deg",       tyDegrees);
            table.put("TargetArea",   targetArea);
            table.put("TagId",        tagId);
            table.put("TotalLatencyMs", totalLatencyMs);
        }

        @Override
        public void fromLog(LogTable table) {
            megaTag2Valid       = table.get("MegaTag2/Valid",       megaTag2Valid);
            megaTag2Pose        = table.get("MegaTag2/Pose",        megaTag2Pose);
            megaTag2Timestamp   = table.get("MegaTag2/Timestamp",   megaTag2Timestamp);
            megaTag2TagCount    = table.get("MegaTag2/TagCount",    megaTag2TagCount);
            megaTag2AvgTagDistM = table.get("MegaTag2/AvgTagDistM", megaTag2AvgTagDistM);

            megaTag1Valid       = table.get("MegaTag1/Valid",       megaTag1Valid);
            megaTag1Pose        = table.get("MegaTag1/Pose",        megaTag1Pose);
            megaTag1Timestamp   = table.get("MegaTag1/Timestamp",   megaTag1Timestamp);
            megaTag1TagCount    = table.get("MegaTag1/TagCount",    megaTag1TagCount);

            hasTargets    = table.get("HasTargets",    hasTargets);
            txDegrees     = table.get("TX_deg",        txDegrees);
            tyDegrees     = table.get("TY_deg",        tyDegrees);
            targetArea    = table.get("TargetArea",    targetArea);
            tagId         = table.get("TagId",         tagId);
            totalLatencyMs = table.get("TotalLatencyMs", totalLatencyMs);
        }
    }
}
