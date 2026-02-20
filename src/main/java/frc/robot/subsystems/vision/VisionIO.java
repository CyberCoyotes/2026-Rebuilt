package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * VisionIO - Hardware abstraction interface for the vision subsystem.
 *
 * WHY AN INTERFACE?
 * This lets us swap between real hardware (Limelight) and simulation
 * without changing any code in VisionSubsystem. During practice at home
 * without the robot, VisionIOSim can pretend to see targets.
 *
 * WHAT THIS DOES:
 * The Limelight camera sees AprilTags on the field and tells us:
 *   1. WHERE the robot is on the field (pose estimation via MegaTag2)
 *   2. WHICH DIRECTION a target is (tx = horizontal angle)
 *   3. HOW FAR a target is (calculated from ty = vertical angle)
 *
 * HOW MEGATAG2 WORKS (simple explanation):
 * - The Limelight sees AprilTag(s) on the field
 * - It knows exactly where every AprilTag is on the field (from a map)
 * - Using the robot's current heading from the gyro + the tag positions,
 *   it calculates where the robot must be on the field
 * - We feed this into WPILib's pose estimator to keep odometry accurate
 * - This corrects for wheel slip and drift that builds up over time
 */
public interface VisionIO {

    /**
     * VisionIOInputs - All data we read from the camera each cycle.
     *
     * NOTE: No @AutoLog here because PoseEstimate is not a loggable type.
     * We log individual fields manually in VisionSubsystem instead.
     */
    class VisionIOInputs {

        // ===== Basic Target Detection =====
        // These tell us if the camera sees anything and where it is

        /** True if the camera sees at least one valid AprilTag */
        public boolean hasTargets = false;

        /**
         * Horizontal angle to the primary target in degrees.
         * Positive = target is to the RIGHT of center
         * Negative = target is to the LEFT of center
         * Used to align the robot rotationally to the target.
         */
        public double horizontalAngleDegrees = 0.0;

        /**
         * Vertical angle to the primary target in degrees.
         * Positive = target is ABOVE center
         * Negative = target is BELOW center
         * Used to calculate distance using camera geometry.
         */
        public double verticalAngleDegrees = 0.0;

        /** Target area as a percentage of the camera image (0.0 to 100.0).
         *  Larger area = closer target. Useful for distance sanity checks. */
        public double targetArea = 0.0;

        /** AprilTag ID of the primary detected tag (-1 if none) */
        public int tagId = -1;

        // ===== MegaTag2 Pose Estimate =====
        // This is the main reason we have vision — accurate field position

        /**
         * Full MegaTag2 pose estimate from the Limelight.
         * Contains: robot Pose2d, timestamp, tag count, average distance, etc.
         * Will be null if no tags are visible or camera is disconnected.
         *
         * MegaTag2 is Limelight's most accurate pose estimation mode.
         * It uses the robot's gyro heading (which we send TO the Limelight)
         * combined with AprilTag positions to calculate field position.
         */
        public PoseEstimate megaTag2Estimate = null;

        /**
         * True if megaTag2Estimate is valid and safe to use for odometry.
         * Checks: not null, has at least one tag, robot isn't spinning too fast.
         */
        public boolean hasValidPoseEstimate = false;

        // ===== Latency =====
        /** Total camera latency in milliseconds (pipeline + capture).
         *  Already accounted for in megaTag2Estimate.timestampSeconds. */
        public double totalLatencyMs = 0.0;
    }

    /**
     * Updates inputs with latest camera data.
     * Called every periodic cycle by VisionSubsystem.
     */
    default void updateInputs(VisionIOInputs inputs) {}

    /**
     * Sends the robot's current heading to the Limelight.
     * REQUIRED for MegaTag2 to work correctly.
     * Must be called before reading pose estimates.
     *
     * @param yawDegrees Robot heading in degrees (from gyro)
     * @param yawRateDps How fast the robot is rotating (degrees per second)
     */
    default void setRobotOrientation(double yawDegrees, double yawRateDps) {}

    /**
     * Sets the active pipeline on the Limelight.
     * Pipeline 0 is typically configured for AprilTag detection.
     */
    default void setPipeline(int pipelineIndex) {}
}