package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * VisionIOInputs — All vision data read from hardware each cycle.
 *
 * Must be a top-level class (not nested) for @AutoLog to generate
 * VisionIOInputsAutoLogged correctly.
 */
@AutoLog
public class VisionIOInputs {

    // ===== Pose Estimate =====

    /** Whether the Limelight returned a valid pose estimate this cycle */
    public boolean poseValid = false;

    /** The estimated robot pose in WPILib Blue Alliance field coordinates */
    public Pose2d estimatedPose = new Pose2d();

    /** Timestamp of the estimate, adjusted for latency (FPGA seconds) */
    public double timestampSeconds = 0.0;

    /** Number of AprilTags used in this estimate */
    public int tagCount = 0;

    /** Average distance from robot to the tags used (meters) */
    public double avgTagDistance = 0.0;

    /** Average tag area across tags used (% of image) */
    public double avgTagArea = 0.0;

    // ===== Basic Target Data =====

    /** Whether the Limelight sees any valid target this cycle */
    public boolean hasTarget = false;

    /** Primary target horizontal angle in degrees (tx) */
    public double txDegrees = 0.0;

    // ===== MegaTag2 Pose Estimate =====

    /** MegaTag2 robot pose [x, y, yawRadians] in WPILib Blue field coordinates */
    public double[] megaTag2Pose = new double[3];

    /** Timestamp of the MegaTag2 estimate in FPGA seconds */
    public double megaTag2TimestampSeconds = 0.0;

    /** Number of tags used in the MegaTag2 estimate (0 = invalid) */
    public int megaTag2TagCount = 0;

    /** Average distance to tags used in MegaTag2 estimate, in meters */
    public double megaTag2AvgTagDist = 0.0;

    // ===== Latency =====

    /** Pipeline processing latency in milliseconds */
    public double pipelineLatencyMs = 0.0;

    /** Total pipeline + capture latency in milliseconds */
    public double totalLatencyMs = 0.0;
}