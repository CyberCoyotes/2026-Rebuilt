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

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    /**
     * Set to true to enable AdvantageKit logging and replay features.
     * This is separate from the Logger features, which are still active when this is false.
     *
     * TODO Toggle on/off for testing
     */
    public static final boolean ENABLE_ADVANTAGEKIT = false;

    public Robot() {

        Logger.recordMetadata("ProjectName", "2026 Rebuilt");

        if (ENABLE_ADVANTAGEKIT) {

            if (isReal()) {
                Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            } else {
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }

            Logger.start();
        }

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        if (ENABLE_ADVANTAGEKIT) {
            m_timeAndJoystickReplay.update();
        }

        CommandScheduler.getInstance().run();

        if (ENABLE_ADVANTAGEKIT) {
            Logger.recordOutput("Robot/BatteryVoltage", RobotController.getBatteryVoltage());
            Logger.recordOutput("Robot/BrownedOut", RobotController.isBrownedOut());
            Logger.recordOutput("Robot/CANBusUtilization", RobotController.getCANStatus().percentBusUtilization);
        }

        // Update game data telemetry (polls FMS for scoring shift data)
        m_robotContainer.updateGameData();

        // Vision is handled entirely by VisionSubsystem via VisionIOLimelight.
        // Do not call LimelightHelpers directly here.
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
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        m_robotContainer.updateSimVision();
    }
}