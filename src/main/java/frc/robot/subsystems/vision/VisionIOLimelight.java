package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;

/**
 * VisionIOLimelight - Real hardware implementation for Limelight 4.
 *
 * This class is intentionally simple. It does three things:
 *   1. Tells the Limelight the robot's current heading (needed for MegaTag2)
 *   2. Reads the MegaTag2 pose estimate (where is the robot on the field?)
 *   3. Reads basic targeting data (tx, ty, ta) for alignment
 *
 * All the complex Limelight communication is handled by LimelightHelpers.
 * This class just calls the right methods and puts data into VisionIOInputs.
 *
 * MEGATAG2 EXPLAINED FOR STUDENTS:
 * Normal odometry drifts over time because of wheel slip.
 * Vision correction works like this:
 *   - We tell Limelight our heading from the gyro (setRobotOrientation)
 *   - Limelight sees AprilTags and calculates where the robot must be
 *   - We feed that position into WPILib's pose estimator
 *   - WPILib blends vision + odometry together for best accuracy
 *
 * The key insight: gyros are accurate for heading but drift for position.
 * AprilTags are accurate for position. Together they're better than either alone.
 */
public class VisionIOLimelight implements VisionIO {

    private final String cameraName;

    /**
     * Maximum rotation speed at which we trust vision pose estimates.
     * When spinning fast, the camera blur makes pose estimates unreliable.
     * 2.0 rotations/second is about 720 degrees/second — pretty fast.
     */
    private static final double MAX_ANGULAR_VELOCITY_RPS = 2.0;

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param cameraName Must match exactly the hostname set in Limelight settings
     *                   (e.g., "limelight-four" not "four" or "limelight")
     */
    public VisionIOLimelight(String cameraName) {
        this.cameraName = cameraName;

        // Warm up the LimelightHelpers NT entry cache.
        // LimelightHelpers creates NetworkTable entries lazily (on first use).
        // Calling this now pays that cost during startup instead of
        // mid-match on the first periodic() call.
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        // Start on pipeline 0 (AprilTag pipeline)
        setPipeline(0);
    }

    /**
     * Sends the robot's heading to the Limelight and reads all vision data.
     * Called every 20ms by VisionSubsystem.
     *
     * ORDER MATTERS:
     * setRobotOrientation must be called BEFORE getBotPoseEstimate_wpiBlue_MegaTag2.
     * The Limelight uses the heading we send to calculate the pose estimate.
     * If we read the estimate first, we'd be using the heading from last cycle.
     */
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Step 1: Read the MegaTag2 pose estimate
        // (orientation was already sent this cycle via setRobotOrientation,
        //  which VisionSubsystem calls before updateInputs)
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        inputs.megaTag2Estimate = estimate;

        // Validate the estimate before marking it usable
        // tagCount > 0 means we actually saw a tag (not just a stale estimate)
        inputs.hasValidPoseEstimate = estimate != null
                && estimate.tagCount > 0;

        inputs.totalLatencyMs = (estimate != null) ? estimate.latency : 0.0;

        // Step 2: Read basic targeting data (used for alignment commands)
        inputs.hasTargets = LimelightHelpers.getTV(cameraName);

        if (inputs.hasTargets) {
            inputs.horizontalAngleDegrees = LimelightHelpers.getTX(cameraName);
            inputs.verticalAngleDegrees = LimelightHelpers.getTY(cameraName);
            inputs.targetArea = LimelightHelpers.getTA(cameraName);
            inputs.tagId = (int) LimelightHelpers.getFiducialID(cameraName);
        } else {
            inputs.horizontalAngleDegrees = 0.0;
            inputs.verticalAngleDegrees = 0.0;
            inputs.targetArea = 0.0;
            inputs.tagId = -1;
        }
    }

    /**
     * Sends robot heading to Limelight for MegaTag2.
     * Use the _NoFlush variant — regular SetRobotOrientation forces a
     * synchronous NT flush every loop which wastes ~5ms.
     */
    @Override
    public void setRobotOrientation(double yawDegrees, double yawRateDps) {
        LimelightHelpers.SetRobotOrientation_NoFlush(
                cameraName,
                yawDegrees,
                yawRateDps,
                0, 0, 0, 0   // pitch/roll not needed for ground robots
        );
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(cameraName, pipelineIndex);
    }
}