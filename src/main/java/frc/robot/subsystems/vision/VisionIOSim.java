package frc.robot.subsystems.vision;

/**
 * VisionIOSim - Simulation implementation of VisionIO for testing without hardware.
 *
 * This class provides fake vision data for testing robot code without a real Limelight.
 * Useful for:
 * - Testing shooter logic at home
 * - Unit testing commands
 * - Simulation mode
 * - Rapid prototyping
 *
 * USAGE:
 * In RobotContainer or Robot.java:
 * ```java
 * if (Robot.isReal()) {
 *     vision = new VisionSubsystem(new VisionIOLimelight("limelight"));
 * } else {
 *     vision = new VisionSubsystem(new VisionIOSim());
 * }
 * ```
 *
 * TODO: Implement realistic simulation data based on robot pose when needed
 */

@SuppressWarnings("unused") // Remove when we approach comp ready code

public class VisionIOSim implements VisionIO {

    private boolean simulateTarget = false;
    private double simulatedDistance = 3.0; // meters
    private double simulatedHorizontalAngle = 0.0; // degrees
    private int simulatedTagId = 7;

    /**
     * Creates a new VisionIOSim with default values (no target).
     */
    public VisionIOSim() {
        // Default: no target visible
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if (simulateTarget) {
            inputs.hasTargets = true;
            inputs.horizontalAngleRadians = Math.toRadians(simulatedHorizontalAngle);
            inputs.verticalAngleRadians = 0.0; // Simplified
            inputs.targetArea = 5.0; // Fake reasonable area
            inputs.tagId = simulatedTagId;
            inputs.pipelineLatencyMs = 11.0;
            inputs.captureLatencyMs = 11.0;
            inputs.totalLatencyMs = 22.0;
        } else {
            // No target
            inputs.hasTargets = false;
            inputs.horizontalAngleRadians = 0.0;
            inputs.verticalAngleRadians = 0.0;
            inputs.targetArea = 0.0;
            inputs.tagId = -1;
        }
    }

    // ===== Methods for controlling simulation (for testing) =====

    /**
     * Simulates seeing a target at a specific distance and angle.
     *
     * @param distanceMeters Distance to target in meters
     * @param horizontalAngleDegrees Horizontal angle to target in degrees
     * @param tagId AprilTag ID
     */
    public void setSimulatedTarget(double distanceMeters, double horizontalAngleDegrees, int tagId) {
        this.simulateTarget = true;
        this.simulatedDistance = distanceMeters;
        this.simulatedHorizontalAngle = horizontalAngleDegrees;
        this.simulatedTagId = tagId;
    }

    /**
     * Simulates losing the target.
     */
    public void clearTarget() {
        this.simulateTarget = false;
    }

    /**
     * Sets whether target is visible.
     *
     * @param visible true to show target, false to hide
     */
    public void setTargetVisible(boolean visible) {
        this.simulateTarget = visible;
    }
}
