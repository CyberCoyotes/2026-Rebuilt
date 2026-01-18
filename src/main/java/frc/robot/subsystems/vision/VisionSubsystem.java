package frc.robot.subsystems.vision;

/*
* VisionSubsystem
* This subsystem handles all vision processing tasks for the robot.
* It interfaces with cameras and processes images to identify targets.
*
* GOAL 1: Stationary shooting with vision alignment
State machines needed:
// ShooterSubsystem
enum ShooterState {
    IDLE,      // Flywheel off, hood home
    SPINUP,    // Flywheel + hood moving to target
    READY,     // Both at target, ready to feed
    EJECT      // Reverse for jams
}

// VisionSubsystem  
enum AlignmentState {
    NO_TARGET,
    TARGET_ACQUIRED,
    ALIGNED,
    LOST_TARGET
}

// IndexerSubsystem (already done)
enum IndexerState {
    IDLE, FLOOR_TRANSPORT, FEEDING, EJECTING
}

Shooting command flow:
// ShootCommand (or CommandGroup):
1. shooter.updateFromVision(vision)  // Set flywheel/hood targets
2. shooter.setState(SPINUP)          // Start spinning up
3. AlignToTargetCommand()            // Drivetrain rotational alignment
4. Wait until: shooter.isReady() && vision.isAligned() && indexer.hasGamePiece()
5. indexer.setState(FEEDING)         // Shoot!

*/

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem {
    enum AlignmentState {
    NO_TARGET,
    TARGET_ACQUIRED,
    ALIGNED,
    LOST_TARGET
    }

    public VisionSubsystem() {
        // Constructor implementation
    }
}
