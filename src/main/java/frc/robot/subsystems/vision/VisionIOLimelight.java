package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

/**
 * VisionIOLimelight — MegaTag2 implementation of VisionIO for the LL4.
 *
 * Every loop:
 *   1. RobotContainer (via VisionSubsystem) calls setRobotOrientation() with the
 *      current gyro yaw so MegaTag2 has a stable heading reference.
 *   2. updateInputs() reads botpose_orb_wpiblue via getBotPoseEstimate_wpiBlue_MegaTag2().
 *   3. VisionSubsystem validates the estimate and feeds it to the pose estimator.
 *
 * IMU Mode strategy (LL4 internal IMU):
 *   - Mode 1 (EXTERNAL_SEED): Used while disabled. Seeds the internal IMU with the
 *     robot's gyro yaw so it knows its starting orientation.
 *   - Mode 4 (INTERNAL_EXTERNAL_ASSIST): Used while enabled. Internal IMU runs at
 *     1kHz for smooth updates; external gyro gently corrects drift over time.
 */
public class VisionIOLimelight implements VisionIO {

    private final String limelightName;

    /**
     * @param limelightName NetworkTable name of the Limelight (e.g. "limelight-four")
     */
    public VisionIOLimelight(String limelightName) {
        this.limelightName = limelightName;
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.SetIMUMode(limelightName, 0);
    }

    /**
     * Sets the LL4 IMU mode.
     * Call with mode 1 while disabled (seeding), mode 4 when enabled (full operation).
     */
    public void setIMUMode(int mode) {
        System.out.println("[Vision] Setting IMU mode to: " + mode);
        LimelightHelpers.SetIMUMode(limelightName, mode);
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(limelightName, yawDegrees, 0, 0, 0, 0, 0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget      = LimelightHelpers.getTV(limelightName);
        inputs.txDegrees      = LimelightHelpers.getTX(limelightName);
        inputs.totalLatencyMs = LimelightHelpers.getLatency_Pipeline(limelightName)
                              + LimelightHelpers.getLatency_Capture(limelightName);

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