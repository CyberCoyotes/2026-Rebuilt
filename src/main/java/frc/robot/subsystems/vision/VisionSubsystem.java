package frc.robot.subsystems.vision;

import java.util.function.BiConsumer;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * VisionSubsystem — MegaTag2-first pose fusion and target tracking.
 *
 * Each periodic cycle:
 *   1. Sends current IMU yaw to Limelight (required for MegaTag2).
 *   2. Reads MegaTag2 pose estimate; feeds it to the drivetrain pose estimator.
 *   3. Falls back to MegaTag1 (2D) when MegaTag2 has no tags.
 *   4. Exposes TX and target validity so AlignAndShootCommand can drive TX → 0.
 *
 * Distance for shot tuning comes from the fused drivetrain pose (hub geometry),
 * not from this subsystem — the drivetrain pose is already vision-corrected once
 * addVisionMeasurement() is called here.
 *
 * Constructor:
 *   io                  — hardware or sim implementation
 *   yawDegrees          — current robot heading supplier (e.g. drivetrain pose rotation)
 *   addVisionMeasurement — drivetrain::addVisionMeasurement(Pose2d, Double)
 */
public class VisionSubsystem extends SubsystemBase {

    private final VisionIO io;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    private final DoubleSupplier yawDegrees;
    private final BiConsumer<Pose2d, Double> addVisionMeasurement;

    private double lastTargetSeenTime = Double.NEGATIVE_INFINITY;

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
            BiConsumer<Pose2d, Double> addVisionMeasurement) {

        this.io = io;
        this.yawDegrees = yawDegrees;
        this.addVisionMeasurement = addVisionMeasurement;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");
        poseSourcePublisher    = table.getStringTopic("PoseSource").publish();
        hasTargetPublisher     = table.getBooleanTopic("HasTarget").publish();
        megaTag2ActivePublisher = table.getBooleanTopic("MegaTag2Active").publish();
        tagIdPublisher         = table.getIntegerTopic("TagID").publish();
        txPublisher            = table.getDoubleTopic("TX_deg").publish();
        targetAreaPublisher    = table.getDoubleTopic("TargetArea").publish();
        latencyPublisher       = table.getDoubleTopic("TotalLatency_ms").publish();

        io.setLEDMode(VisionIO.LEDMode.PIPELINE_DEFAULT);
        io.setPipeline(Constants.Vision.APRILTAG_PIPELINE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs, yawDegrees.getAsDouble());
        Logger.processInputs("Vision", inputs);

        // Feed pose estimate to drivetrain — MegaTag2 preferred, MegaTag1 as fallback
        if (inputs.megaTag2Valid) {
            addVisionMeasurement.accept(arrayToPose(inputs.megaTag2Pose), inputs.megaTag2Timestamp);
            lastTargetSeenTime = Timer.getFPGATimestamp();
        } else if (inputs.megaTag1Valid) {
            addVisionMeasurement.accept(arrayToPose(inputs.megaTag1Pose), inputs.megaTag1Timestamp);
            lastTargetSeenTime = Timer.getFPGATimestamp();
        }

        logTelemetry();
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * True when the camera actively sees a valid target this cycle.
     * TX is only reliable when this is true — check before calling getTX().
     */
    public boolean hasFreshTarget() {
        return inputs.hasTargets && isHubTag(inputs.tagId);
    }

    /**
     * True when the most recent pose measurement came from MegaTag2 (vs MegaTag1).
     * Useful for dashboard diagnostics and shot confidence decisions.
     */
    public boolean isMegaTag2Active() {
        return inputs.megaTag2Valid;
    }

    /**
     * Horizontal angle from camera center to primary tag in degrees.
     * Positive = target is to the right of center.
     * Zero when no target. Always check hasFreshTarget() before using.
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
     * Checks whether the given tag ID belongs to the current alliance's hub.
     * Blue hub: {18, 19, 20, 21, 24, 25, 26, 27}
     * Red hub:  {2, 3, 4, 5, 8, 9, 10, 11}
     * Defaults to blue if FMS has not reported alliance.
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

    /** Converts a [x_m, y_m, yaw_deg] array to Pose2d. */
    private static Pose2d arrayToPose(double[] arr) {
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
