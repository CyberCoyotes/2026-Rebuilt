package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * VisionSubsystem — MegaTag2 pose estimation and hub targeting.
 *
 * RESPONSIBILITIES:
 *   1. Calls setRobotOrientation() every loop so MegaTag2 has a stable heading reference.
 *   2. Validates incoming pose estimates and feeds good ones to the drivetrain's
 *      SwerveDrivePoseEstimator via the poseConsumer callback.
 *   3. On the FIRST valid pose estimate, performs a hard resetPose() via resetPoseCallback
 *      so the robot snaps to its real field position instead of blending from field center.
 *   4. Exposes getDistanceToHub() and getAngleToHub() so HubTrackingCommand and
 *      future commands can aim purely from robot pose without touching raw camera data.
 *
 * WIRING (in RobotContainer):
 *   vision = new VisionSubsystem(
 *       new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME),
 *       drivetrain::addVisionMeasurement,
 *       () -> drivetrain.resetPoseToVision(vision.getLastAcceptedPose()),
 *       () -> drivetrain.getState().Pose,
 *       () -> drivetrain.getState().Pose.getRotation().getDegrees()
 *   );
 *
 * POSE VALIDATION:
 *   Estimates are rejected if:
 *     - tagCount < MIN_TAG_COUNT
 *     - avgTagDistance > MAX_DISTANCE_METERS
 *     - The estimated position is outside the WPILib field boundary
 */
public class VisionSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // WPILib field boundary (2025 Reefscape field is 17.55m x 8.02m)
    // -------------------------------------------------------------------------
    private static final double FIELD_LENGTH_M = 17.55;
    private static final double FIELD_WIDTH_M  =  8.02;

    // -------------------------------------------------------------------------
    // Hardware & callbacks
    // -------------------------------------------------------------------------

    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();

    /** Called with (pose, timestampSeconds, stdDevs) when a valid estimate arrives. */
    private final PoseEstimateConsumer poseConsumer;

    /**
     * Called once on the first valid tag sighting to hard-reset the drivetrain pose.
     * This prevents the Kalman filter from blending against the wrong starting pose.
     */
    private final Runnable resetPoseCallback;

    /** Supplies the current robot pose (for hub angle/distance calculation). */
    private final Supplier<Pose2d> poseSupplier;

    /** Supplies the current robot yaw in degrees (for setRobotOrientation). */
    private final Supplier<Double> yawSupplier;

    // -------------------------------------------------------------------------
    // NetworkTables publishers
    // -------------------------------------------------------------------------

    private final NetworkTable     visionTable;
    private final BooleanPublisher poseValidPublisher;
    private final BooleanPublisher hasTargetPublisher;
    private final DoublePublisher  distanceToHubPublisher;
    private final DoublePublisher  angleToHubPublisher;
    private final DoublePublisher  tagCountPublisher;
    private final DoublePublisher  avgTagDistPublisher;
    private final DoublePublisher  latencyPublisher;
    private final StringPublisher  estimatedPosePublisher;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private Pose2d  lastAcceptedPose = new Pose2d();
    private int     acceptedCount    = 0;
    private int     rejectedCount    = 0;
    private boolean hasResetPose     = false;

    // -------------------------------------------------------------------------
    // Functional interface for pose consumer
    // -------------------------------------------------------------------------

    @FunctionalInterface
    public interface PoseEstimateConsumer {
        void accept(Pose2d pose, double timestampSeconds,
                    edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3,
                    edu.wpi.first.math.numbers.N1> stdDevs);
    }

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * @param io                 VisionIO hardware implementation
     * @param poseConsumer       Callback to feed validated pose estimates to the drivetrain
     * @param resetPoseCallback  Called once on first valid tag to hard-reset the drivetrain pose
     * @param poseSupplier       Supplies the current robot Pose2d (from drivetrain odometry)
     * @param yawSupplier        Supplies the current robot yaw in degrees (from drivetrain IMU)
     */
    public VisionSubsystem(
            VisionIO io,
            PoseEstimateConsumer poseConsumer,
            Runnable resetPoseCallback,
            Supplier<Pose2d> poseSupplier,
            Supplier<Double> yawSupplier) {

        this.io                = io;
        this.poseConsumer      = poseConsumer;
        this.resetPoseCallback = resetPoseCallback;
        this.poseSupplier      = poseSupplier;
        this.yawSupplier       = yawSupplier;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        visionTable = inst.getTable("Vision");

        poseValidPublisher     = visionTable.getBooleanTopic("PoseValid").publish();
        hasTargetPublisher     = visionTable.getBooleanTopic("HasTarget").publish();
        distanceToHubPublisher = visionTable.getDoubleTopic("DistanceToHub_m").publish();
        angleToHubPublisher    = visionTable.getDoubleTopic("AngleToHub_deg").publish();
        tagCountPublisher      = visionTable.getDoubleTopic("TagCount").publish();
        avgTagDistPublisher    = visionTable.getDoubleTopic("AvgTagDist_m").publish();
        latencyPublisher       = visionTable.getDoubleTopic("TotalLatency_ms").publish();
        estimatedPosePublisher = visionTable.getStringTopic("EstimatedPose").publish();

        io.setPipeline(Constants.Vision.APRILTAG_PIPELINE);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        io.setRobotOrientation(yawSupplier.get());
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        if (inputs.poseValid && isEstimateAcceptable()) {
            if (!hasResetPose) {
                lastAcceptedPose = inputs.estimatedPose;
                resetPoseCallback.run();
                hasResetPose = true;
            }

            poseConsumer.accept(
                inputs.estimatedPose,
                inputs.timestampSeconds,
                buildStdDevs()
            );
            lastAcceptedPose = inputs.estimatedPose;
            acceptedCount++;
        } else if (inputs.poseValid) {
            rejectedCount++;
        }

        publishTelemetry();
    }

    // -------------------------------------------------------------------------
    // Pose estimate validation
    // -------------------------------------------------------------------------

    private boolean isEstimateAcceptable() {
        if (inputs.tagCount < Constants.Vision.MIN_TAG_COUNT) return false;
        if (inputs.avgTagDistance > Constants.Vision.MAX_DISTANCE_METERS) return false;

        double x = inputs.estimatedPose.getX();
        double y = inputs.estimatedPose.getY();
        if (x < 0 || x > FIELD_LENGTH_M || y < 0 || y > FIELD_WIDTH_M) return false;

        return true;
    }

    /**
     * Builds standard deviation matrix for the pose estimator.
     * Lower values = more trust in vision. Scales with tag distance.
     * With more tags, we trust the measurement more (lower stddev).
     */
    private edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3,
            edu.wpi.first.math.numbers.N1> buildStdDevs() {

        double xyStdDev;
        if (inputs.tagCount >= 2) {
            xyStdDev = 0.3;
        } else {
            xyStdDev = 0.5 + (inputs.avgTagDistance * 0.1);
        }

        // Rotation is always handled by the IMU in MegaTag2 — trust it fully
        double rotStdDev = 9999.0;

        return edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }

    // -------------------------------------------------------------------------
    // Public API — hub targeting
    // -------------------------------------------------------------------------

    /**
     * Returns the straight-line distance from the current robot pose to the
     * center of the hub in meters. Returns -1.0 if no valid pose is available.
     */
    public double getDistanceToHub() {
        Pose2d pose = poseSupplier.get();
        if (pose == null) return -1.0;
        return pose.getTranslation().getDistance(Constants.Vision.HUB_CENTER_BLUE);
    }

    /**
     * Returns the signed angle error in degrees the robot needs to rotate to face the hub.
     * Positive = hub is to the left, negative = right. Returns 0.0 if no pose available.
     */
    public double getAngleToHub() {
        Pose2d pose = poseSupplier.get();
        if (pose == null) return 0.0;

        Translation2d toHub     = Constants.Vision.HUB_CENTER_BLUE.minus(pose.getTranslation());
        Rotation2d angleToHub   = new Rotation2d(toHub.getX(), toHub.getY());
        Rotation2d currentAngle = pose.getRotation();

        return currentAngle.minus(angleToHub).getDegrees();
    }

    /**
     * Returns true if the vision system has accepted at least one pose estimate.
     */
    public boolean hasPose() {
        return acceptedCount > 0;
    }

    /**
     * Returns the last accepted robot pose from vision. Check hasPose() before relying on this.
     */
    public Pose2d getLastAcceptedPose() {
        return lastAcceptedPose;
    }

    /**
     * Returns true if the Limelight currently sees any target.
     */
    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    /**
     * Returns the raw tx from the Limelight in degrees.
     * Only use this as a last resort fallback — prefer getAngleToHub().
     */
    public double getRawTxDegrees() {
        return inputs.txDegrees;
    }

    /**
     * Returns true if the currently visible tag is a hub target for the active alliance.
     *
     * Defaults to blue alliance if DriverStation has not yet reported alliance color
     * (e.g., during pre-match or practice mode without FMS).
     *
     * NOTE: Hub tag ID ranges must be verified against the 2026 game manual.
     * Blue: Constants.Vision.BLUE_HUB_MIN/MAX_TAG_ID
     * Red:  Constants.Vision.RED_HUB_MIN/MAX_TAG_ID
     */
    public boolean isHubTarget() {
        // Tag ID is not tracked in the current MegaTag2 inputs — always returns false
        // until a tagId field is added to VisionIOInputs and populated in VisionIOLimelight.
        return false;
    }

    // =========================================================================
    // PUBLIC API - Camera Control
    // =========================================================================

    /**
     * Sets the active vision pipeline.
     *
     * @param pipelineIndex Pipeline index (0-9)
     */
    public void setPipeline(int pipelineIndex) {
        io.setPipeline(pipelineIndex);
    }

    /**
     * Sets the LED mode.
     *
     * @param mode LED mode to set
     */
    public void setLEDMode(VisionIO.LEDMode mode) {
        io.setLEDMode(mode);
    }

    // =========================================================================
    // PUBLIC API - MegaTag2 accessors (for external use if needed)
    // =========================================================================

    /**
     * Returns true if MegaTag2 produced a valid pose estimate this cycle.
     */
    public boolean hasMegaTag2Estimate() {
        return inputs.megaTag2TagCount > 0;
    }

    /**
     * Gets the MegaTag2 pose estimate as a Pose2d in WPILib Blue field coordinates.
     * Check hasMegaTag2Estimate() before calling this.
     */
    public Pose2d getMegaTag2Pose() {
        return new Pose2d(
            inputs.megaTag2Pose[0],
            inputs.megaTag2Pose[1],
            new Rotation2d(inputs.megaTag2Pose[2])
        );
    }

    /**
     * Gets the timestamp of the MegaTag2 estimate in FPGA seconds.
     */
    public double getMegaTag2Timestamp() {
        return inputs.megaTag2TimestampSeconds;
    }

    /**
     * Gets the average distance to tags used in the MegaTag2 estimate.
     */
    public double getMegaTag2AvgTagDist() {
        return inputs.megaTag2AvgTagDist;
    }

    // =========================================================================
    // TELEMETRY
    // =========================================================================

    /**
     * Publishes telemetry to NetworkTables (Elastic dashboard) and AdvantageKit.
     */
    private void publishTelemetry() {
        poseValidPublisher.set(inputs.poseValid);
        hasTargetPublisher.set(inputs.hasTarget);
        distanceToHubPublisher.set(getDistanceToHub());
        angleToHubPublisher.set(getAngleToHub());
        tagCountPublisher.set(inputs.tagCount);
        avgTagDistPublisher.set(inputs.avgTagDistance);
        latencyPublisher.set(inputs.totalLatencyMs);
        estimatedPosePublisher.set(
            inputs.poseValid
                ? String.format("(%.2f, %.2f)",
                    inputs.estimatedPose.getX(),
                    inputs.estimatedPose.getY())
                : "invalid"
        );

        Logger.recordOutput("Vision/AcceptedCount",    acceptedCount);
        Logger.recordOutput("Vision/RejectedCount",    rejectedCount);
        Logger.recordOutput("Vision/HasResetPose",     hasResetPose);
        Logger.recordOutput("Vision/DistanceToHub",    getDistanceToHub());
        Logger.recordOutput("Vision/AngleToHub",       getAngleToHub());
        Logger.recordOutput("Vision/LastAcceptedPose", lastAcceptedPose);
    }
}