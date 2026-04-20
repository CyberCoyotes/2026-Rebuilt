package frc.robot.subsystems.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * VisionSubsystem — sensor data provider for vision-assisted driving.
 *
 * Responsibilities:
 *   - Calls SetRobotOrientation (with yaw + yaw rate) before each MT2 read so inputs
 *     are logged with accurate orientation data.
 *   - Logs MT2/MT1 pose estimates and raw TX/TV to AKit for replay.
 *   - Exposes hasFreshTarget() and getTX() so AlignAndShootCommand can drive TX → 0.
 *
 * Pose fusion (addVisionMeasurement) is NOT done here — Robot.robotPeriodic()
 * owns that with the distance-scaled std dev formula and omega gate.  Keeping
 * fusion in one place avoids double-counting measurements in the Kalman filter.
 *
 * Constructor params:
 *   io                — hardware or sim implementation
 *   yawDegrees        — current robot heading supplier
 *   yawRateDegPerSec  — current robot yaw rate supplier (improves MT2 accuracy)
 */
public class VisionSubsystem extends SubsystemBase {

    private final VisionIO io;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    private final DoubleSupplier yawDegrees;
    private final DoubleSupplier yawRateDegPerSec;

    // ── NT publishers for Elastic dashboard ───────────────────────────────────
    private final StringPublisher  poseSourcePublisher;
    private final BooleanPublisher hasTargetPublisher;
    private final BooleanPublisher megaTag2ActivePublisher;
    private final IntegerPublisher tagIdPublisher;
    private final DoublePublisher  txPublisher;
    private final DoublePublisher  targetAreaPublisher;
    private final DoublePublisher  latencyPublisher;

    public VisionSubsystem(
            VisionIO io,
            DoubleSupplier yawDegrees,
            DoubleSupplier yawRateDegPerSec) {

        this.io = io;
        this.yawDegrees = yawDegrees;
        this.yawRateDegPerSec = yawRateDegPerSec;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");
        poseSourcePublisher     = table.getStringTopic("PoseSource").publish();
        hasTargetPublisher      = table.getBooleanTopic("HasTarget").publish();
        megaTag2ActivePublisher = table.getBooleanTopic("MegaTag2Active").publish();
        tagIdPublisher          = table.getIntegerTopic("TagID").publish();
        txPublisher             = table.getDoubleTopic("TX_deg").publish();
        targetAreaPublisher     = table.getDoubleTopic("TargetArea").publish();
        latencyPublisher        = table.getDoubleTopic("TotalLatency_ms").publish();

        io.setLEDMode(VisionIO.LEDMode.PIPELINE_DEFAULT);
        io.setPipeline(Constants.Vision.APRILTAG_PIPELINE);
    }

    @Override
    public void periodic() {
        // SetRobotOrientation is called inside updateInputs (VisionIOLimelight) so
        // AKit-logged MT2 reads use the same fresh orientation that Robot.java fuses.
        io.updateInputs(inputs, yawDegrees.getAsDouble(), yawRateDegPerSec.getAsDouble());
        Logger.processInputs("Vision", inputs);

        // Pose fusion is handled by Robot.robotPeriodic() with weighted std devs.
        // Do NOT call addVisionMeasurement here — it would double-count measurements.

        logTelemetry();
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * True when the camera actively sees a valid hub target this cycle.
     * Always check before calling getTX() — TX is only valid when this is true.
     */
    public boolean hasFreshTarget() {
        return inputs.hasTargets && isHubTag(inputs.tagId);
    }

    /**
     * True when the latest pose estimate used MegaTag2 (vs MegaTag1 / no tag).
     * Useful for dashboard confidence display.
     */
    public boolean isMegaTag2Active() {
        return inputs.megaTag2Valid;
    }

    /**
     * Horizontal angle from camera center to primary tag in degrees.
     * Positive = target is to the right.  Zero when no target is visible.
     * Check hasFreshTarget() before using for control.
     */
    public double getTX() {
        return inputs.txDegrees;
    }

    /** Primary AprilTag ID visible this cycle, -1 if none. */
    public int getTagId() {
        return inputs.tagId;
    }

    // =========================================================================
    // Internal helpers
    // =========================================================================

    /**
     * Returns true if the given tag ID belongs to the current alliance's hub.
     * Blue hub: {18, 19, 20, 21, 24, 25, 26, 27}
     * Red hub:  {2, 3, 4, 5, 8, 9, 10, 11}
     * Defaults to blue when FMS has not reported.
     */
    private boolean isHubTag(int id) {
        if (id < 0) return false;
        int[] hubTags;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            hubTags = Constants.Vision.RED_HUB_TAG_IDS;
        } else {
            hubTags = Constants.Vision.BLUE_HUB_TAG_IDS;
        }
        for (int hubTag : hubTags) {
            if (id == hubTag) return true;
        }
        return false;
    }

    /** Converts a [x_m, y_m, yaw_deg] inputs array to Pose2d. */
    static Pose2d arrayToPose(double[] arr) {
        return new Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[2]));
    }

    private void logTelemetry() {
        String poseSource = inputs.megaTag2Valid ? "MegaTag2"
                          : inputs.megaTag1Valid ? "MegaTag1"
                          : "None";

        poseSourcePublisher.set(poseSource);
        hasTargetPublisher.set(hasFreshTarget());
        megaTag2ActivePublisher.set(inputs.megaTag2Valid);
        tagIdPublisher.set(inputs.tagId);
        txPublisher.set(inputs.txDegrees);
        targetAreaPublisher.set(inputs.targetArea);
        latencyPublisher.set(inputs.totalLatencyMs);

        Logger.recordOutput("Vision/PoseSource", poseSource);
        Logger.recordOutput("Vision/TX_deg",     inputs.txDegrees);
        Logger.recordOutput("Vision/HasTarget",  hasFreshTarget());
    }
}
