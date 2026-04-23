package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * VisionIOLimelight - Limelight hardware implementation of VisionIO.
 *
 * Uses LimelightHelpers static methods for all pose estimation reads.
 * SetRobotOrientation() is called first each cycle — MegaTag2 requires it.
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
 * - botpose_orb_wpiblue: Robot pose (MegaTag2, WPILib blue-origin frame) [x, y, z, roll, pitch, yaw]
 * - tl: Pipeline latency (ms)
 * - cl: Capture latency (ms)
 */
public class VisionIOLimelight implements VisionIO {

    private final String limelightName;

    // LED and pipeline control via raw NT (LimelightHelpers doesn't expose setLEDMode)
    private final NetworkTableEntry ledModeEntry;
    private final NetworkTableEntry pipelineEntry;

    public VisionIOLimelight(String limelightName) {
        // Get the Limelight's NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

        // Cache NetworkTable entries for performance (avoid repeated lookups)
        validEntry = limelightTable.getEntry("tv");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        taEntry = limelightTable.getEntry("ta");
        tagIdEntry = limelightTable.getEntry("tid");
        botposeEntry = limelightTable.getEntry("botpose_orb_wpiblue");
        pipelineLatencyEntry = limelightTable.getEntry("tl");
        captureLatencyEntry = limelightTable.getEntry("cl");
        ledModeEntry = limelightTable.getEntry("ledMode");
        pipelineEntry = limelightTable.getEntry("pipeline");

        // Initialize Limelight to known state
        setLEDMode(LEDMode.PIPELINE_DEFAULT);
        setPipeline(0);
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs, double robotYawDegrees, double robotYawRateDegPerSec) {

        // Must be sent before reading MegaTag2 — seeds the rotation-locked estimate.
        // Yaw rate improves accuracy during rotation; the other rates are unnecessary.
        LimelightHelpers.SetRobotOrientation(limelightName, robotYawDegrees, robotYawRateDegPerSec, 0, 0, 0, 0);

        // ── MegaTag2 (primary) ──────────────────────────────────────────────────
        LimelightHelpers.PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        inputs.megaTag2Valid = mt2 != null && mt2.tagCount > 0;
        if (inputs.megaTag2Valid) {
            inputs.megaTag2Pose        = poseToArray(mt2);
            inputs.megaTag2Timestamp   = mt2.timestampSeconds;
            inputs.megaTag2TagCount    = mt2.tagCount;
            inputs.megaTag2AvgTagDistM = mt2.avgTagDist;
        } else {
            inputs.megaTag2Pose        = new double[3];
            inputs.megaTag2Timestamp   = 0.0;
            inputs.megaTag2TagCount    = 0;
            inputs.megaTag2AvgTagDistM = 0.0;
        }

        // ── MegaTag1 fallback (2D, no IMU required) ────────────────────────────
        LimelightHelpers.PoseEstimate mt1 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        inputs.megaTag1Valid = mt1 != null && mt1.tagCount > 0;
        if (inputs.megaTag1Valid) {
            inputs.megaTag1Pose      = poseToArray(mt1);
            inputs.megaTag1Timestamp = mt1.timestampSeconds;
            inputs.megaTag1TagCount  = mt1.tagCount;
        } else {
            inputs.megaTag1Pose      = new double[3];
            inputs.megaTag1Timestamp = 0.0;
            inputs.megaTag1TagCount  = 0;
        }

        // ── Raw target data ────────────────────────────────────────────────────
        inputs.hasTargets = LimelightHelpers.getTV(limelightName);
        if (inputs.hasTargets) {
            inputs.txDegrees  = LimelightHelpers.getTX(limelightName);
            inputs.tyDegrees  = LimelightHelpers.getTY(limelightName);
            inputs.targetArea = LimelightHelpers.getTA(limelightName);
            inputs.tagId      = (int) LimelightHelpers.getFiducialID(limelightName);
        } else {
            inputs.txDegrees  = 0.0;
            inputs.tyDegrees  = 0.0;
            inputs.targetArea = 0.0;
            inputs.tagId      = -1;
        }

        // Use MegaTag2 latency when available, else MegaTag1, else 0
        inputs.totalLatencyMs = inputs.megaTag2Valid ? mt2.latency
                              : inputs.megaTag1Valid ? mt1.latency
                              : 0.0;
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        pipelineEntry.setNumber(pipelineIndex);
    }

    @Override
    public void setLEDMode(LEDMode mode) {
        ledModeEntry.setNumber(mode.value);
    }

    /** Converts a PoseEstimate to [x_m, y_m, yaw_deg] for LoggableInputs. */
    private static double[] poseToArray(LimelightHelpers.PoseEstimate est) {
        return new double[]{
            est.pose.getX(),
            est.pose.getY(),
            est.pose.getRotation().getDegrees()
        };
    }
}
