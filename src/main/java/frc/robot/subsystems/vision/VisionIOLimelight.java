package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * VisionIOLimelight — MegaTag2 implementation of VisionIO for the LL4.
 *
 * Every loop:
 *   1. RobotContainer (via VisionSubsystem) calls setRobotOrientation() with the
 *      current gyro yaw so MegaTag2 has a stable heading reference.
 *   2. updateInputs() reads botpose_orb_wpiblue via getBotPoseEstimate_wpiBlue_MegaTag2().
 *   3. VisionSubsystem validates the estimate and feeds it to the pose estimator.
 *
 * Note: MegaTag2 fixes the rotation component of the pose using the IMU, so the
 * returned yaw matches the robot's gyro yaw. Only X/Y translation is solved from
 * the tags. This makes it very stable even with a single tag visible.
 */
public class VisionIOLimelight implements VisionIO {

    private final String limelightName;

    /**
     * @param limelightName NetworkTable name of the Limelight (e.g. "limelight-four")
     */
    public VisionIOLimelight(String limelightName) {
        this.limelightName = limelightName;
        LimelightHelpers.setPipelineIndex(limelightName, 0);
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
        // Send current IMU yaw to the Limelight so MegaTag2 can use it.
        // Yaw rate and other angular rates are not needed — pass zeros.
        LimelightHelpers.SetRobotOrientation(limelightName, yawDegrees, 0, 0, 0, 0, 0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Read raw tx and target valid flag from NT — always available
        inputs.hasTarget   = LimelightHelpers.getTV(limelightName);
        inputs.txDegrees   = LimelightHelpers.getTX(limelightName);
        inputs.totalLatencyMs = LimelightHelpers.getLatency_Pipeline(limelightName)
                              + LimelightHelpers.getLatency_Capture(limelightName);

        // Read MegaTag2 pose estimate
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (estimate == null || estimate.pose == null) {
            inputs.poseValid = false;
            return;
        }

        inputs.poseValid        = true;
        inputs.estimatedPose    = estimate.pose;
        inputs.timestampSeconds = estimate.timestampSeconds;
        inputs.tagCount         = estimate.tagCount;
        inputs.avgTagDistance   = estimate.avgTagDist;
        inputs.avgTagArea       = estimate.avgTagArea;
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }
}