package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * VisionSubsystem - Manages vision processing for robot alignment and shooting.
 *
 * PRIMARY PURPOSE (GOAL 1): Stationary shooting with vision alignment
 * - Provides distance and angle data to shooter for flywheel/hood adjustments
 * - Provides horizontal angle data to drivetrain for rotational alignment
 * - Tracks alignment state for command coordination
 *
 * ARCHITECTURE:
 * - Uses VisionIO interface for hardware abstraction
 * - Tracks AlignmentState for coordination with other subsystems
 * - Provides calculated values (distance, angles) derived from raw camera data
 * - Integrates with AdvantageKit for logging and replay
 *
 * STATE MACHINE:
 * - NO_TARGET: No valid AprilTag visible
 * - TARGET_ACQUIRED: Tag visible, not yet aligned
 * - ALIGNED: Robot aligned within tolerance, ready to shoot
 * - LOST_TARGET: Had target but lost it (uses last known values briefly)
 *
 * USAGE EXAMPLE:
 * // In ShooterSubsystem:
 * double distance = vision.getDistanceToTargetMeters();
 * double velocity = calculateVelocityFromDistance(distance);
 *
 * // In AlignToTargetCommand:
 * double angleError = vision.getHorizontalAngleDegrees();
 * drivetrain.rotate(angleError * kP);
 *
 * // In ShootCommand:
 * if (vision.isAligned() && shooter.isReady()) {
 *     indexer.feed();
 * }
 */
public class VisionSubsystem extends SubsystemBase {

    // ===== Hardware Interface =====
    private final VisionIO io;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    // ===== State Tracking =====
    /**
     * Alignment state for the vision system.
     * Tracks whether we have a target and if we're aligned to it.
     */
    public enum AlignmentState {
        /** No valid target visible */
        NO_TARGET,

        /** Target visible, robot not yet aligned */
        TARGET_ACQUIRED,

        /** Robot aligned to target within tolerance, ready to shoot */
        ALIGNED,

        /** Had a target but lost it (grace period using last known values) */
        LOST_TARGET
    }

    private AlignmentState currentState = AlignmentState.NO_TARGET;
    private AlignmentState previousState = AlignmentState.NO_TARGET;

    // ===== Last Known Good Data =====
    // When we lose a target, we briefly hold onto the last known values
    // This prevents sudden jumps and allows smooth recovery
    private double lastKnownDistance = 0.0;
    private double lastKnownHorizontalAngle = 0.0;
    private double lastTargetSeenTime = 0.0;

    /**
     * Creates a new VisionSubsystem.
     *
     * @param io The VisionIO hardware interface implementation
     */
    public VisionSubsystem(VisionIO io) {
        this.io = io;

        // Initialize Limelight to known state
        io.setLEDMode(VisionIO.LEDMode.PIPELINE_DEFAULT);
        io.setPipeline(Constants.Vision.APRILTAG_PIPELINE);
    }

    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        // Update state machine
        updateAlignmentState();

        // Log telemetry
        logTelemetry();
    }

    // =========================================================================
    // STATE MACHINE
    // =========================================================================

    /**
     * Updates the alignment state based on current vision data.
     * Called automatically in periodic().
     */
    private void updateAlignmentState() {
        previousState = currentState;

        if (inputs.hasTargets && isTargetValid()) {
            // We have a valid target
            lastTargetSeenTime = Timer.getFPGATimestamp();

            // Update last known good values
            lastKnownDistance = calculateDistance();
            lastKnownHorizontalAngle = getHorizontalAngleDegrees();

            // Check if we're aligned
            if (isWithinAlignmentTolerance()) {
                currentState = AlignmentState.ALIGNED;
            } else {
                currentState = AlignmentState.TARGET_ACQUIRED;
            }
        } else {
            // No valid target
            double timeSinceLastTarget = Timer.getFPGATimestamp() - lastTargetSeenTime;

            if (timeSinceLastTarget < Constants.Vision.TARGET_TIMEOUT_SECONDS &&
                previousState != AlignmentState.NO_TARGET) {
                // Grace period: use last known values
                currentState = AlignmentState.LOST_TARGET;
            } else {
                // Definitely lost target
                currentState = AlignmentState.NO_TARGET;
                lastKnownDistance = 0.0;
                lastKnownHorizontalAngle = 0.0;
            }
        }

        // Log state transitions
        if (currentState != previousState) {
            Logger.recordOutput("Vision/StateTransition",
                previousState.name() + " -> " + currentState.name());
        }
    }

    /**
     * Checks if the current target is valid (correct tag ID, reasonable distance, etc.)
     */
    private boolean isTargetValid() {
        // Check tag ID is in valid range
        if (inputs.tagId < Constants.Vision.MIN_VALID_TAG_ID ||
            inputs.tagId > Constants.Vision.MAX_VALID_TAG_ID) {
            return false;
        }

        // Check target area is reasonable (not too small = too far)
        if (inputs.targetArea < Constants.Vision.MIN_TARGET_AREA_PERCENT) {
            return false;
        }

        // Check calculated distance is reasonable
        double distance = calculateDistance();
        if (distance > Constants.Vision.MAX_DISTANCE_METERS || distance < 0.1) {
            return false;
        }

        return true;
    }

    /**
     * Checks if robot is aligned within tolerance for shooting.
     */
    private boolean isWithinAlignmentTolerance() {
        double horizontalError = Math.abs(getHorizontalAngleDegrees());
        return horizontalError <= Constants.Vision.ALIGNMENT_TOLERANCE_DEGREES;
    }

    // =========================================================================
    // PUBLIC API - State Queries
    // =========================================================================

    /**
     * Gets the current alignment state.
     *
     * @return Current alignment state
     */
    public AlignmentState getAlignmentState() {
        return currentState;
    }

    /**
     * Checks if a valid target is currently visible.
     *
     * @return true if valid target is visible
     */
    public boolean hasTarget() {
        return currentState == AlignmentState.TARGET_ACQUIRED ||
               currentState == AlignmentState.ALIGNED;
    }

    /**
     * Checks if robot is aligned to target and ready to shoot.
     *
     * @return true if aligned within tolerance
     */
    public boolean isAligned() {
        return currentState == AlignmentState.ALIGNED;
    }

    /**
     * Gets the AprilTag ID currently being tracked.
     *
     * @return Tag ID, or -1 if no valid target
     */
    public int getTagId() {
        return inputs.tagId;
    }

    // =========================================================================
    // PUBLIC API - Calculated Values for Shooter
    // =========================================================================

    /**
     * Calculates distance to target in meters using camera geometry.
     *
     * Uses the formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
     *
     * For LOST_TARGET state, returns last known distance for smooth transitions.
     *
     * @return Distance to target in meters, or 0 if no target
     */
    public double getDistanceToTargetMeters() {
        if (currentState == AlignmentState.NO_TARGET) {
            return 0.0;
        }

        if (currentState == AlignmentState.LOST_TARGET) {
            return lastKnownDistance; // Use last known value during grace period
        }

        return calculateDistance();
    }

    /**
     * Internal distance calculation from camera geometry.
     */
    private double calculateDistance() {
        double heightDiff = Constants.Vision.APRILTAG_HEIGHT_METERS -
                          Constants.Vision.CAMERA_HEIGHT_METERS;

        double verticalAngleDegrees = Units.radiansToDegrees(inputs.verticalAngleRadians);
        double angleToTarget = Constants.Vision.CAMERA_ANGLE_DEGREES + verticalAngleDegrees;

        return Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTarget)));
    }

    /**
     * Gets distance to target in centimeters (for compatibility with older code).
     *
     * @return Distance in centimeters
     */
    public double getDistanceToTargetCM() {
        return getDistanceToTargetMeters() * 100.0;
    }

    // =========================================================================
    // PUBLIC API - Angle Values for Drivetrain Alignment
    // =========================================================================

    /**
     * Gets horizontal angle to target in degrees.
     *
     * Positive = target is to the right
     * Negative = target is to the left
     *
     * For LOST_TARGET state, returns last known angle for smooth transitions.
     *
     * @return Horizontal angle in degrees, or 0 if no target
     */
    public double getHorizontalAngleDegrees() {
        if (currentState == AlignmentState.NO_TARGET) {
            return 0.0;
        }

        if (currentState == AlignmentState.LOST_TARGET) {
            return lastKnownHorizontalAngle; // Use last known value
        }

        return Units.radiansToDegrees(inputs.horizontalAngleRadians);
    }

    /**
     * Gets horizontal angle to target in radians.
     *
     * @return Horizontal angle in radians
     */
    public double getHorizontalAngleRadians() {
        return Units.degreesToRadians(getHorizontalAngleDegrees());
    }

    /**
     * Gets vertical angle to target in degrees.
     *
     * @return Vertical angle in degrees
     */
    public double getVerticalAngleDegrees() {
        return Units.radiansToDegrees(inputs.verticalAngleRadians);
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
    // TELEMETRY
    // =========================================================================

    /**
     * Logs comprehensive telemetry to SmartDashboard and AdvantageKit.
     */
    private void logTelemetry() {
        // State information
        SmartDashboard.putString("Vision/State", currentState.name());
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putBoolean("Vision/IsAligned", isAligned());

        // Raw values
        SmartDashboard.putNumber("Vision/TagID", getTagId());
        SmartDashboard.putNumber("Vision/TargetArea", inputs.targetArea);

        // Calculated values
        SmartDashboard.putNumber("Vision/Distance_m", getDistanceToTargetMeters());
        SmartDashboard.putNumber("Vision/Distance_cm", getDistanceToTargetCM());
        SmartDashboard.putNumber("Vision/HorizontalAngle_deg", getHorizontalAngleDegrees());
        SmartDashboard.putNumber("Vision/VerticalAngle_deg", getVerticalAngleDegrees());

        // Latency
        SmartDashboard.putNumber("Vision/TotalLatency_ms", inputs.totalLatencyMs);

        // AdvantageKit logging
        Logger.recordOutput("Vision/State", currentState.name());
        Logger.recordOutput("Vision/HasTarget", hasTarget());
        Logger.recordOutput("Vision/IsAligned", isAligned());
        Logger.recordOutput("Vision/Distance_m", getDistanceToTargetMeters());
        Logger.recordOutput("Vision/HorizontalAngle_deg", getHorizontalAngleDegrees());
    }
}
