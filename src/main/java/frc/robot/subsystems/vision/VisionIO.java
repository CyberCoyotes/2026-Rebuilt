package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * VisionIO — Hardware abstraction for MegaTag2-based pose estimation.
 *
 * This interface is purpose-built for the LL4 MegaTag2 workflow:
 *   1. RobotContainer calls SetRobotOrientation() every loop with the current gyro yaw.
 *   2. VisionIOLimelight reads the resulting botpose_orb_wpiblue estimate.
 *   3. VisionSubsystem validates the estimate and feeds it to SwerveDrivePoseEstimator.
 *
 * The old tx/ty/distance approach is gone. All targeting math (distance to hub,
 * angle to hub) is now done in VisionSubsystem using the robot's pose and the
 * known hub field position from Constants.Vision.HUB_CENTER_BLUE.
 */
public interface VisionIO {

    /**
     * Updates inputs from the camera hardware.
     * Called every periodic loop by VisionSubsystem.
     */
    default void updateInputs(VisionIOInputs inputs) {}

    /**
     * Sends the robot's current heading to the Limelight so MegaTag2
     * can produce a rotationally-stable pose estimate.
     * Must be called every loop BEFORE reading pose estimates.
     *
     * @param yawDegrees Current robot yaw from the drivetrain IMU (degrees, CCW positive)
     */
    default void setRobotOrientation(double yawDegrees) {}

    /**
     * Sets the active pipeline.
     *
     * @param pipelineIndex Pipeline index (0-9)
     */
    default void setPipeline(int pipelineIndex) {}

    // -------------------------------------------------------------------------
    // Inputs container
    // -------------------------------------------------------------------------

    class VisionIOInputs implements LoggableInputs {

        // Whether the Limelight returned a valid pose estimate this cycle
        public boolean poseValid = false;

        // The estimated robot pose in WPILib Blue Alliance field coordinates
        public Pose2d estimatedPose = new Pose2d();

        // Timestamp of the estimate, adjusted for latency (FPGA seconds)
        public double timestampSeconds = 0.0;

        // Number of tags used in this estimate
        public int tagCount = 0;

        // Average distance from robot to the tags used (meters)
        public double avgTagDistance = 0.0;

        // Average tag area across tags used (% of image)
        public double avgTagArea = 0.0;

        // Whether the Limelight sees any target at all (tv)
        public boolean hasTarget = false;

        // Primary target horizontal angle (degrees) — used as fallback if pose invalid
        public double txDegrees = 0.0;

        // Pipeline + capture latency (ms)
        public double totalLatencyMs = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("PoseValid",       poseValid);
            table.put("EstimatedPose",   estimatedPose);
            table.put("TimestampSeconds", timestampSeconds);
            table.put("TagCount",        tagCount);
            table.put("AvgTagDistance",  avgTagDistance);
            table.put("AvgTagArea",      avgTagArea);
            table.put("HasTarget",       hasTarget);
            table.put("TxDegrees",       txDegrees);
            table.put("TotalLatencyMs",  totalLatencyMs);
        }

        @Override
        public void fromLog(LogTable table) {
            poseValid        = table.get("PoseValid",        poseValid);
            estimatedPose    = table.get("EstimatedPose",    estimatedPose);
            timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
            tagCount         = table.get("TagCount",         tagCount);
            avgTagDistance   = table.get("AvgTagDistance",   avgTagDistance);
            avgTagArea       = table.get("AvgTagArea",       avgTagArea);
            hasTarget        = table.get("HasTarget",        hasTarget);
            txDegrees        = table.get("TxDegrees",        txDegrees);
            totalLatencyMs   = table.get("TotalLatencyMs",   totalLatencyMs);
        }
    }
}