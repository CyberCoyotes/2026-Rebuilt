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

        // Vision pose fusion — weighted std devs so trust drops with distance.
        // SetRobotOrientation is handled by VisionSubsystem (with yaw + yaw rate)
        // so we read immediately; the orientation was set earlier this cycle.
        // Theta std dev = 9999999 for both modes: gyro always owns heading.
        double omegaRps = Units.radiansToRotations(
                m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond);
        boolean omegaOk = Math.abs(omegaRps) < Constants.Vision.OMEGA_FILTER_MAX_RPS;

        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.LIMELIGHT4_NAME);
        if (mt2 != null && mt2.tagCount > 0 && omegaOk) {
            double xyStdDev = Constants.Vision.VISION_MT2_STD_DEV_COEFF * Math.pow(mt2.avgTagDist, 2.0);
            m_robotContainer.drivetrain.addVisionMeasurement(
                    mt2.pose, mt2.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        } else if ((mt2 == null || mt2.tagCount == 0) && omegaOk) {
            // MT1 fallback (2D) — lower trust, higher std devs
            var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.LIMELIGHT4_NAME);
            if (mt1 != null && mt1.tagCount > 0) {
                double xyStdDev = Constants.Vision.VISION_MT1_STD_DEV_COEFF * Math.pow(mt1.avgTagDist, 2.0);
                m_robotContainer.drivetrain.addVisionMeasurement(
                        mt1.pose, mt1.timestampSeconds,
                        VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
            }
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
