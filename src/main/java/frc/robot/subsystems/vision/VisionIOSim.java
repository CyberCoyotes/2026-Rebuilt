package frc.robot.subsystems.vision;

/**
 * VisionIOSim - Simulation implementation of VisionIO.
 *
 * Returns no vision targets. Prevents subsystems that depend on vision from
 * crashing in simulation while keeping the pose-fusion path exercisable.
 */
public class VisionIOSim implements VisionIO {

    @Override
    public void updateInputs(VisionIOInputs inputs, double robotYawDegrees, double robotYawRateDegPerSec) {
        inputs.megaTag2Valid = false;
        inputs.megaTag1Valid = false;
        inputs.hasTargets = false;
        inputs.tagId = -1;
    }
}
