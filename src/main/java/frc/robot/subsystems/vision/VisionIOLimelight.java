package frc.robot.subsystems.vision;

<<<<<<< Updated upstream
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
=======
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.LimelightHelpers;
>>>>>>> Stashed changes

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

<<<<<<< Updated upstream
    private final String limelightName;
=======
    // ===== Camera Identity =====
    private final String limelightName;

    // ===== NetworkTable Entries =====
    private final NetworkTable limelightTable;
    private final NetworkTableEntry validEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry botposeEntry;
    private final NetworkTableEntry pipelineLatencyEntry;
    private final NetworkTableEntry captureLatencyEntry;
    private final NetworkTableEntry ledModeEntry;
    private final NetworkTableEntry pipelineEntry;
>>>>>>> Stashed changes

    /**
     * @param limelightName NetworkTable name of the Limelight (e.g. "limelight-four")
     */
    public VisionIOLimelight(String limelightName) {
        this.limelightName = limelightName;
<<<<<<< Updated upstream
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.SetIMUMode(limelightName, 0);
    }
=======
        // Get the Limelight's NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
>>>>>>> Stashed changes

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

<<<<<<< Updated upstream
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
=======
        // Check if we have a valid target
        inputs.hasTargets = validEntry.getDouble(0.0) > 0.5; // Use 0.5 threshold for robustness

        if (inputs.hasTargets) {
            // Read target data and convert to standard units (radians)
            inputs.horizontalAngleRadians = Units.degreesToRadians(txEntry.getDouble(0.0));
            inputs.verticalAngleRadians = Units.degreesToRadians(tyEntry.getDouble(0.0));
            inputs.targetArea = taEntry.getDouble(0.0);

            // Read AprilTag specific data
            inputs.tagId = (int) tagIdEntry.getDouble(-1.0);

            // Clone botpose array for defensive copy (prevents external modification)
            double[] rawBotpose = botposeEntry.getDoubleArray(new double[6]);
            inputs.botpose = rawBotpose.clone();
       } else {
            // No target detected - reset all values to zero/invalid
            inputs.horizontalAngleRadians = 0.0;
            inputs.verticalAngleRadians = 0.0;
            inputs.targetArea = 0.0;
            inputs.tagId = -1;
            inputs.botpose = new double[6]; // All zeros
        }

        // ===== MegaTag2 Pose Estimate =====
        // Note: SetRobotOrientation() must be called before this each loop,
        // which VisionSubsystem handles via setRobotOrientation()
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 != null && mt2.tagCount > 0) {
            inputs.megaTag2Pose = new double[]{
                mt2.pose.getX(),
                mt2.pose.getY(),
                mt2.pose.getRotation().getRadians()
            };
            inputs.megaTag2TimestampSeconds = mt2.timestampSeconds;
            inputs.megaTag2TagCount = mt2.tagCount;
            inputs.megaTag2AvgTagDist = mt2.avgTagDist;
        } else {
            inputs.megaTag2Pose = new double[3];
            inputs.megaTag2TimestampSeconds = 0.0;
            inputs.megaTag2TagCount = 0;
            inputs.megaTag2AvgTagDist = 0.0;
        }
>>>>>>> Stashed changes
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }
<<<<<<< Updated upstream
=======

    @Override
    public void setLEDMode(LEDMode mode) {
        ledModeEntry.setNumber(mode.value);
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(limelightName,
            yawDegrees, 0, 0, 0, 0, 0);
    }
>>>>>>> Stashed changes
}