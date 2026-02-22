package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * VisionIOSim — Simulation implementation of VisionIO.
 *
 * Simulates a Limelight 4 running MegaTag2 by generating fake but physically
 * plausible pose estimates and tx values based on the robot's current simulated
 * position relative to the hub.
 *
 * Swap into RobotContainer automatically using:
 *   Robot.isSimulation()
 *       ? new VisionIOSim()
 *       : new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME)
 *
 * SIMULATION BEHAVIOUR:
 *   - Pose estimate is the robot's true sim pose with small Gaussian noise added
 *   - tx is calculated from the angle between the robot heading and the hub
 *   - Tag count is 2 when within 2.5m, 1 when further, 0 when out of range/FOV
 *   - Latency is fixed at a realistic 30ms
 *   - Detection only occurs within MAX_DETECTION_RANGE_M and DETECTION_FOV_DEG
 *   - Small random dropout chance simulates real-world frame drops
 *
 * WIRING:
 *   Store the VisionIOSim reference separately so you can call setSimRobotPose()
 *   from simulationPeriodic() in Robot.java:
 *
 *     // In RobotContainer field:
 *     private final VisionIOSim visionIOSim = new VisionIOSim();
 *
 *     // In RobotContainer constructor (sim path):
 *     vision = new VisionSubsystem(
 *         Robot.isSimulation()
 *             ? visionIOSim
 *             : new VisionIOLimelight(Constants.Vision.LIMELIGHT4_NAME),
 *         drivetrain::addVisionMeasurement,
 *         () -> drivetrain.getState().Pose,
 *         () -> drivetrain.getState().RawHeading.getDegrees()
 *     );
 *
 *     // In Robot.java simulationPeriodic():
 *     robotContainer.updateSimVision();
 *
 *     // In RobotContainer:
 *     public void updateSimVision() {
 *         visionIOSim.setSimRobotPose(drivetrain.getState().Pose);
 *     }
 */
public class VisionIOSim implements VisionIO {

    // -------------------------------------------------------------------------
    // Simulation parameters
    // -------------------------------------------------------------------------

    /** Maximum distance at which the sim camera can detect hub tags (meters) */
    private static final double MAX_DETECTION_RANGE_M = 5.0;

    /** Half-angle of the simulated camera field of view (degrees) */
    private static final double DETECTION_FOV_DEG = 30.0;

    /** Simulated total pipeline + capture latency (ms) */
    private static final double SIM_LATENCY_MS = 30.0;

    /** Gaussian noise standard deviation on X/Y pose estimate (meters) */
    private static final double POSE_NOISE_METERS = 0.02;

    /** Gaussian noise standard deviation on tx (degrees) */
    private static final double TX_NOISE_DEG = 0.3;

    /** Probability of a random frame dropout per loop tick (0.0 to 1.0) */
    private static final double DROPOUT_PROBABILITY = 0.02;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    // Default starting pose — robot near the middle of the field facing the hub
    private Pose2d simRobotPose = new Pose2d(8.0, 4.0, new Rotation2d());

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public VisionIOSim() {}

    // -------------------------------------------------------------------------
    // VisionIO implementation
    // -------------------------------------------------------------------------

    @Override
    public void setRobotOrientation(double yawDegrees) {
        // No-op in simulation — yaw is already captured in simRobotPose
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Translation2d hubCenter = Constants.Vision.HUB_CENTER_BLUE;
        Translation2d robotPos  = simRobotPose.getTranslation();

        // Distance from robot to hub center
        double distanceToHub = robotPos.getDistance(hubCenter);

        // True angle from robot's heading to the hub (degrees)
        Translation2d toHub      = hubCenter.minus(robotPos);
        Rotation2d    angleToHub = new Rotation2d(toHub.getX(), toHub.getY());
        double        txTrue     = angleToHub.minus(simRobotPose.getRotation()).getDegrees();

        // Normalise to [-180, 180]
        while (txTrue >  180.0) txTrue -= 360.0;
        while (txTrue < -180.0) txTrue += 360.0;

        // Determine visibility
        boolean inRange = distanceToHub <= MAX_DETECTION_RANGE_M;
        boolean inFov   = Math.abs(txTrue) <= DETECTION_FOV_DEG;
        boolean dropout = Math.random() < DROPOUT_PROBABILITY;
        boolean canSee  = inRange && inFov && !dropout;

        inputs.totalLatencyMs = SIM_LATENCY_MS;

        if (!canSee) {
            inputs.hasTarget = false;
            inputs.poseValid = false;
            inputs.txDegrees = 0.0;
            inputs.tagCount  = 0;
            return;
        }

        // Simulate raw tx with noise
        inputs.hasTarget = true;
        inputs.txDegrees = txTrue + gaussianNoise(TX_NOISE_DEG);

        // Simulate pose estimate with noise
        double noisyX = simRobotPose.getX() + gaussianNoise(POSE_NOISE_METERS);
        double noisyY = simRobotPose.getY() + gaussianNoise(POSE_NOISE_METERS);

        inputs.poseValid        = true;
        inputs.estimatedPose    = new Pose2d(noisyX, noisyY, simRobotPose.getRotation());
        inputs.timestampSeconds = Timer.getFPGATimestamp() - (SIM_LATENCY_MS / 1000.0);
        inputs.tagCount         = distanceToHub < 2.5 ? 2 : 1;
        inputs.avgTagDistance   = distanceToHub;
        inputs.avgTagArea       = Math.min(100.0, 5.0 / (distanceToHub * distanceToHub));
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        // No-op in simulation
    }

    // -------------------------------------------------------------------------
    // Sim control — call from Robot.simulationPeriodic() via RobotContainer
    // -------------------------------------------------------------------------

    /**
     * Updates the simulated robot pose so VisionIOSim knows where the robot is.
     * Call this every simulationPeriodic() tick:
     *
     *   visionIOSim.setSimRobotPose(drivetrain.getState().Pose);
     *
     * @param pose Current simulated robot pose in WPILib Blue field coordinates
     */
    public void setSimRobotPose(Pose2d pose) {
        if (pose != null) {
            this.simRobotPose = pose;
        }
    }

    /**
     * Returns the current simulated robot pose (useful for debugging).
     */
    public Pose2d getSimRobotPose() {
        return simRobotPose;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /**
     * Generates a Gaussian-distributed random value using the Box-Muller transform.
     *
     * @param stdDev Standard deviation of the distribution
     * @return Random sample from N(0, stdDev)
     */
    private double gaussianNoise(double stdDev) {
        double u1 = Math.max(1e-10, Math.random()); // avoid log(0)
        double u2 = Math.random();
        double z  = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        return z * stdDev;
    }
}