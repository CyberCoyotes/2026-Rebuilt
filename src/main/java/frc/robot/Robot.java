// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.LimelightHelpers;

@SuppressWarnings("unused") // Suppress warnings for unused right now

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        
    Logger.recordMetadata("ProjectName", "2026 Rebuilt"); // Set a metadata value
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    

    m_robotContainer = new RobotContainer();

    }

    @Override
    public void robotPeriodic() {
  
            m_timeAndJoystickReplay.update();

        CommandScheduler.getInstance().run();

            // Log system health data for post-match analysis
            Logger.recordOutput("Robot/BatteryVoltage", RobotController.getBatteryVoltage());
            Logger.recordOutput("Robot/BrownedOut", RobotController.isBrownedOut());
            Logger.recordOutput("Robot/CANBusUtilization", RobotController.getCANStatus().percentBusUtilization);

        // Update game data telemetry (polls FMS for scoring shift data)
        m_robotContainer.updateGameData();

        // Vision pose estimation: std devs scale with distance so trust drops as tags get farther away.
        // Theta std dev is set very high (9999999) since MegaTag2 uses the gyro for heading.
        var driveState = m_robotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(Constants.Vision.LIMELIGHT4_NAME, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.LIMELIGHT4_NAME);
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            double dist = llMeasurement.avgTagDist; // meters
            double xyStdDev = 0.05 * Math.pow(dist, 2.0); // trust drops fast with distance
            m_robotContainer.drivetrain.addVisionMeasurement(
                    llMeasurement.pose,
                    llMeasurement.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        // CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
