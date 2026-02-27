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

<<<<<<< Updated upstream
    // -------------------------------------------------------------------------
    // Inputs container
    // -------------------------------------------------------------------------

=======
   /**
     * Controls the camera's LED mode.
     *
     * @param mode The LED mode to set
     */
    default void setLEDMode(LEDMode mode) {}

    /**
     * Sends the robot's current yaw to the Limelight for MegaTag2.
     * Must be called every loop before reading the MegaTag2 pose estimate.
     *
     * @param yawDegrees Current robot yaw in degrees from the drivetrain gyro
     */
    default void setRobotOrientation(double yawDegrees) {}

    /**
     * LED control modes for vision cameras.
     */
    enum LEDMode {        /** Use the LED mode specified by the current pipeline */
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
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
        // Whether the Limelight sees any target at all (tv)
        public boolean hasTarget = false;
=======
        // ===== MegaTag2 Pose Estimate =====
        /** MegaTag2 robot pose [x, y, yawRadians] in WPILib Blue field coordinates */
        public double[] megaTag2Pose = new double[3];

        /** Timestamp of the MegaTag2 estimate in FPGA seconds */
        public double megaTag2TimestampSeconds = 0.0;

        /** Number of tags used in the MegaTag2 estimate (0 = invalid) */
        public int megaTag2TagCount = 0;

        /** Average distance to tags used in MegaTag2 estimate, in meters */
        public double megaTag2AvgTagDist = 0.0;

        // ===== Latency Data =====
        /** Pipeline processing latency in milliseconds */
        public double pipelineLatencyMs = 0.0;
>>>>>>> Stashed changes

        // Primary target horizontal angle (degrees) — used as fallback if pose invalid
        public double txDegrees = 0.0;

        // Pipeline + capture latency (ms)
        public double totalLatencyMs = 0.0;

        @Override
        public void toLog(LogTable table) {
<<<<<<< Updated upstream
            table.put("PoseValid",       poseValid);
            table.put("EstimatedPose",   estimatedPose);
            table.put("TimestampSeconds", timestampSeconds);
            table.put("TagCount",        tagCount);
            table.put("AvgTagDistance",  avgTagDistance);
            table.put("AvgTagArea",      avgTagArea);
            table.put("HasTarget",       hasTarget);
            table.put("TxDegrees",       txDegrees);
            table.put("TotalLatencyMs",  totalLatencyMs);
=======
            table.put("Timestamp", timestamp);
            table.put("HasTargets", hasTargets);
            table.put("HorizontalAngleRad", horizontalAngleRadians);
            table.put("VerticalAngleRad", verticalAngleRadians);
            table.put("TargetArea", targetArea);
            table.put("TagId", tagId);
            table.put("Botpose", botpose);
            table.put("MegaTag2Pose", megaTag2Pose);
            table.put("MegaTag2TimestampSeconds", megaTag2TimestampSeconds);
            table.put("MegaTag2TagCount", megaTag2TagCount);
            table.put("MegaTag2AvgTagDist", megaTag2AvgTagDist);
            table.put("PipelineLatencyMs", pipelineLatencyMs);
            table.put("CaptureLatencyMs", captureLatencyMs);
            table.put("TotalLatencyMs", totalLatencyMs);
>>>>>>> Stashed changes
        }

        @Override
        public void fromLog(LogTable table) {
<<<<<<< Updated upstream
            poseValid        = table.get("PoseValid",        poseValid);
            estimatedPose    = table.get("EstimatedPose",    estimatedPose);
            timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
            tagCount         = table.get("TagCount",         tagCount);
            avgTagDistance   = table.get("AvgTagDistance",   avgTagDistance);
            avgTagArea       = table.get("AvgTagArea",       avgTagArea);
            hasTarget        = table.get("HasTarget",        hasTarget);
            txDegrees        = table.get("TxDegrees",        txDegrees);
            totalLatencyMs   = table.get("TotalLatencyMs",   totalLatencyMs);
=======
            timestamp = table.get("Timestamp", timestamp);
            hasTargets = table.get("HasTargets", hasTargets);
            horizontalAngleRadians = table.get("HorizontalAngleRad", horizontalAngleRadians);
            verticalAngleRadians = table.get("VerticalAngleRad", verticalAngleRadians);
            targetArea = table.get("TargetArea", targetArea);
            tagId = table.get("TagId", tagId);
            botpose = table.get("Botpose", botpose);
            megaTag2Pose = table.get("MegaTag2Pose", megaTag2Pose);
            megaTag2TimestampSeconds = table.get("MegaTag2TimestampSeconds", megaTag2TimestampSeconds);
            megaTag2TagCount = table.get("MegaTag2TagCount", megaTag2TagCount);
            megaTag2AvgTagDist = table.get("MegaTag2AvgTagDist", megaTag2AvgTagDist);
            pipelineLatencyMs = table.get("PipelineLatencyMs", pipelineLatencyMs);
            captureLatencyMs = table.get("CaptureLatencyMs", captureLatencyMs);
            totalLatencyMs = table.get("TotalLatencyMs", totalLatencyMs);
>>>>>>> Stashed changes
        }
    }
}