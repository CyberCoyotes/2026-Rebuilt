package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

import org.littletonrobotics.junction.Logger;

/**
 * VisionSubsystem - Manages vision processing for field localization and target alignment.
 *
 * WHAT THIS DOES:
 * 1. ODOMETRY CORRECTION (most important):
 *    The robot tracks its position on the field using wheel encoders + gyro.
 *    But wheels slip, so position drifts over time.
 *    Vision uses AprilTags to periodically correct that drift.
 *    This is critical for accurate autonomous paths.
 *
 * 2. TARGET ALIGNMENT:
 *    Provides horizontal angle (tx) data so the drivetrain can rotate
 *    to face a target precisely before shooting.
 *
 * 3. DISTANCE CALCULATION:
 *    Uses camera geometry to estimate distance to target from the
 *    vertical angle (ty). Used by shooter to set hood angle and RPM.
 *
 * HOW TO USE:
 *   // In Robot.java robotPeriodic() — replaces the old LimelightHelpers block:
 *   m_robotContainer.vision.updateOdometry(m_robotContainer.drivetrain);
 *
 *   // In commands — check alignment:
 *   if (vision.hasTarget() && vision.getHorizontalAngleDegrees() < 2.0) { ... }
 *
 *   // For shooter distance:
 *   double distance = vision.getDistanceToTargetMeters();
 *   shooter.updateFromDistance(distance);
 */
public class VisionSubsystem extends SubsystemBase {

    // ===== Hardware Interface =====
    private final VisionIO io;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    // ===== Alignment State Machine =====
    public enum AlignmentState {
        NO_TARGET,        // Camera sees nothing
        TARGET_ACQUIRED,  // Sees a tag but not aligned yet
        ALIGNED           // Horizontal angle is within tolerance
    }

    private AlignmentState currentState = AlignmentState.NO_TARGET;

    // ===== Periodic Counter (for throttling dashboard updates) =====
    private int periodicCounter = 0;

    // ===== NetworkTables Publishers for Elastic Dashboard =====
    private final NetworkTable visionTable;
    private final StringPublisher statePublisher;
    private final BooleanPublisher hasTargetPublisher;
    private final BooleanPublisher isAlignedPublisher;
    private final BooleanPublisher hasValidPosePublisher;
    private final IntegerPublisher tagIdPublisher;
    private final DoublePublisher horizontalAnglePublisher;
    private final DoublePublisher verticalAnglePublisher;
    private final DoublePublisher distanceMetersPublisher;
    private final DoublePublisher latencyPublisher;

    /**
     * Creates a new VisionSubsystem.
     *
     * @param io Hardware implementation (VisionIOLimelight for real robot,
     *           VisionIOSim for testing without hardware)
     */
    public VisionSubsystem(VisionIO io) {
        this.io = io;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        visionTable = inst.getTable("Vision");

        statePublisher = visionTable.getStringTopic("State").publish();
        hasTargetPublisher = visionTable.getBooleanTopic("HasTarget").publish();
        isAlignedPublisher = visionTable.getBooleanTopic("IsAligned").publish();
        hasValidPosePublisher = visionTable.getBooleanTopic("HasValidPose").publish();
        tagIdPublisher = visionTable.getIntegerTopic("TagID").publish();
        horizontalAnglePublisher = visionTable.getDoubleTopic("HorizontalAngle_deg").publish();
        verticalAnglePublisher = visionTable.getDoubleTopic("VerticalAngle_deg").publish();
        distanceMetersPublisher = visionTable.getDoubleTopic("Distance_m").publish();
        latencyPublisher = visionTable.getDoubleTopic("TotalLatency_ms").publish();
    }

    // =========================================================================
    // PERIODIC
    // =========================================================================

    @Override
    public void periodic() {
        // NOTE: setRobotOrientation is NOT called here.
        // It's called inside updateOdometry() in Robot.java robotPeriodic(),
        // which runs BEFORE the CommandScheduler (and therefore before this periodic).
        // That ordering ensures orientation is always fresh before we read pose data.

        // Read all camera data
        io.updateInputs(inputs);

        // Update our state machine
        updateAlignmentState();

        // Publish to dashboard at 10Hz (every 5th cycle)
        // Vision data doesn't change fast enough to need 50Hz updates
        if (++periodicCounter % 5 == 0) {
            publishTelemetry();
        }

        // AdvantageKit logging (always runs, independent of dashboard throttle)
        Logger.recordOutput("Vision/State", currentState.name());
        Logger.recordOutput("Vision/HasTarget", inputs.hasTargets);
        Logger.recordOutput("Vision/HasValidPose", inputs.hasValidPoseEstimate);
        Logger.recordOutput("Vision/HorizontalAngle_deg", inputs.horizontalAngleDegrees);
        Logger.recordOutput("Vision/Distance_m", getDistanceToTargetMeters());
    }

    // =========================================================================
    // ODOMETRY UPDATE — called from Robot.java robotPeriodic()
    // =========================================================================

    /**
     * Sends robot heading to the Limelight and feeds any valid pose estimate
     * into the drivetrain's pose estimator.
     *
     * CALL THIS FROM Robot.java robotPeriodic() BEFORE CommandScheduler.run().
     * That ensures orientation is sent to the Limelight before this subsystem's
     * periodic() reads the resulting pose estimate.
     *
     * WHY HERE INSTEAD OF periodic()?
     * We need the current drivetrain state (heading, rotation rate) to send to
     * the Limelight. Passing the drivetrain as a parameter here is cleaner than
     * storing a reference to it in VisionSubsystem permanently, since vision
     * doesn't "own" the drivetrain — it just reads from it occasionally.
     *
     * @param drivetrain The swerve drivetrain (used to get heading and add measurement)
     */
    public void updateOdometry(TunerConstants.CommandSwerveDrivetrain drivetrain) {
        // Step 1: Get current robot heading from drivetrain
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        // Step 2: Send heading to Limelight (required for MegaTag2)
        // The Limelight uses our gyro heading to calculate where we are on the field.
        // _NoFlush avoids a costly synchronous NT write every loop.
        io.setRobotOrientation(headingDeg, omegaRps * 360.0); // convert RPS → deg/s

        // Step 3: If we have a valid pose estimate, feed it to odometry
        // The spinning check filters out poses captured while the robot was rotating fast,
        // because motion blur reduces accuracy.
        if (inputs.hasValidPoseEstimate && Math.abs(omegaRps) < 2.0) {
            drivetrain.addVisionMeasurement(
                    inputs.megaTag2Estimate.pose,
                    inputs.megaTag2Estimate.timestampSeconds
            );
        }
    }

    // =========================================================================
    // STATE MACHINE
    // =========================================================================

    private void updateAlignmentState() {
        if (!inputs.hasTargets) {
            currentState = AlignmentState.NO_TARGET;
            return;
        }

        if (isWithinAlignmentTolerance()) {
            currentState = AlignmentState.ALIGNED;
        } else {
            currentState = AlignmentState.TARGET_ACQUIRED;
        }
    }

    private boolean isWithinAlignmentTolerance() {
        return Math.abs(inputs.horizontalAngleDegrees) <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES;
    }

    // =========================================================================
    // PUBLIC API — Use these in commands and other subsystems
    // =========================================================================

    /** True if camera currently sees a valid AprilTag. */
    public boolean hasTarget() {
        return currentState == AlignmentState.TARGET_ACQUIRED
                || currentState == AlignmentState.ALIGNED;
    }

    /** True if robot is rotated within alignment tolerance of the target. */
    public boolean isAligned() {
        return currentState == AlignmentState.ALIGNED;
    }

    /** True if we have a fresh, valid pose estimate to feed to odometry. */
    public boolean hasValidPoseEstimate() {
        return inputs.hasValidPoseEstimate;
    }

    /** Current alignment state. */
    public AlignmentState getAlignmentState() {
        return currentState;
    }

    /**
     * Horizontal angle to the primary target in degrees.
     * Positive = target is to the RIGHT, negative = to the LEFT.
     * Use this to drive rotational alignment: rotate until this is near zero.
     */
    public double getHorizontalAngleDegrees() {
        return inputs.horizontalAngleDegrees;
    }

    /**
     * Vertical angle to the primary target in degrees.
     * Used internally for distance calculation.
     */
    public double getVerticalAngleDegrees() {
        return inputs.verticalAngleDegrees;
    }

    /** Which AprilTag is currently being tracked (-1 if none). */
    public int getTagId() {
        return inputs.tagId;
    }

    /**
     * Estimated distance to the target in meters, using camera geometry.
     *
     * HOW THIS WORKS:
     * If you know:
     *   - How high the camera is mounted (CAMERA_HEIGHT_METERS)
     *   - How high the target is on the field (APRILTAG_HEIGHT_METERS)
     *   - The camera's tilt angle (CAMERA_ANGLE_DEGREES)
     *   - The vertical angle from camera to target (ty from Limelight)
     *
     * Then: distance = heightDifference / tan(cameraAngle + ty)
     *
     * This is basic trigonometry — the same math you'd use to find the
     * horizontal distance to something if you know its height and angle.
     *
     * @return Distance in meters, or 0.0 if no target visible
     */
    public double getDistanceToTargetMeters() {
        if (!inputs.hasTargets) {
            return 0.0;
        }

        double heightDiff = Constants.Vision.APRILTAG_HEIGHT_METERS
                - Constants.Vision.CAMERA_HEIGHT_METERS;

        double totalAngleDeg = Constants.Vision.CAMERA_ANGLE_DEGREES
                + inputs.verticalAngleDegrees;

        // Avoid division by zero if angle is flat (shouldn't happen in practice)
        if (Math.abs(totalAngleDeg) < 0.1) {
            return 0.0;
        }

        return Math.abs(heightDiff / Math.tan(Math.toRadians(totalAngleDeg)));
    }

    // =========================================================================
    // TELEMETRY (10Hz)
    // =========================================================================

    private void publishTelemetry() {
        statePublisher.set(currentState.name());
        hasTargetPublisher.set(hasTarget());
        isAlignedPublisher.set(isAligned());
        hasValidPosePublisher.set(inputs.hasValidPoseEstimate);
        tagIdPublisher.set(inputs.tagId);
        horizontalAnglePublisher.set(inputs.horizontalAngleDegrees);
        verticalAnglePublisher.set(inputs.verticalAngleDegrees);
        distanceMetersPublisher.set(getDistanceToTargetMeters());
        latencyPublisher.set(inputs.totalLatencyMs);
    }
}