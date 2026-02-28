package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

/**
 * VisionIOLimelight — MegaTag2 implementation of VisionIO for the LL4.
 *
 * Every loop:
 *   1. VisionSubsystem calls setRobotOrientation() with the current gyro yaw so
 *      MegaTag2 has a stable heading reference.
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

    // ===== Camera Identity =====
    private final String limelightName;

    // ===== NetworkTable Entries =====
    private final NetworkTable limelightTable;
    private final NetworkTableEntry ledModeEntry;
    private final NetworkTableEntry pipelineEntry;

    /**
     * @param limelightName NetworkTable name of the Limelight (e.g. "limelight-four")
     */
    public VisionIOLimelight(String limelightName) {
        this.limelightName = limelightName;

        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
        ledModeEntry   = limelightTable.getEntry("ledMode");
        pipelineEntry  = limelightTable.getEntry("pipeline");
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
        // ===== Basic target data =====
        inputs.hasTarget      = LimelightHelpers.getTV(limelightName);
        inputs.txDegrees      = LimelightHelpers.getTX(limelightName);
        inputs.totalLatencyMs = LimelightHelpers.getLatency_Pipeline(limelightName)
                              + LimelightHelpers.getLatency_Capture(limelightName);
        inputs.pipelineLatencyMs = LimelightHelpers.getLatency_Pipeline(limelightName);

        // ===== MegaTag2 Pose Estimate =====
        // Note: setRobotOrientation() must be called before this each loop,
        // which VisionSubsystem handles in its periodic().
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 != null && mt2.tagCount > 0) {
            inputs.poseValid             = true;
            inputs.estimatedPose         = mt2.pose;
            inputs.timestampSeconds      = mt2.timestampSeconds;
            inputs.tagCount              = mt2.tagCount;
            inputs.avgTagDistance        = mt2.avgTagDist;
            inputs.megaTag2Pose          = new double[]{
                mt2.pose.getX(),
                mt2.pose.getY(),
                mt2.pose.getRotation().getRadians()
            };
            inputs.megaTag2TimestampSeconds = mt2.timestampSeconds;
            inputs.megaTag2TagCount         = mt2.tagCount;
            inputs.megaTag2AvgTagDist       = mt2.avgTagDist;
        } else {
            inputs.poseValid                = false;
            inputs.tagCount                 = 0;
            inputs.avgTagDistance           = 0.0;
            inputs.megaTag2Pose             = new double[3];
            inputs.megaTag2TimestampSeconds = 0.0;
            inputs.megaTag2TagCount         = 0;
            inputs.megaTag2AvgTagDist       = 0.0;
        }
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    @Override
    public void setLEDMode(LEDMode mode) {
        ledModeEntry.setNumber(mode.value);
    }

    
}