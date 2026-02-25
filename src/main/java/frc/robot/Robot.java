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

    /**
     * Set to true to enable AdvantageKit logging and replay features.
     * TODO: Toggle on before competition.
     */
    public static final boolean ENABLE_ADVANTAGEKIT = false;

    // Only instantiated when AdvantageKit is enabled — used for timestamp and joystick replay.
    private final HootAutoReplay m_timeAndJoystickReplay;

    public Robot() {
        if (ENABLE_ADVANTAGEKIT) {
            Logger.recordMetadata("ProjectName", "2026 Rebuilt");

            if (isReal()) {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            } else {
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }

            Logger.start();
            m_timeAndJoystickReplay = new HootAutoReplay()
                .withTimestampReplay()
                .withJoystickReplay();
        } else {
            m_timeAndJoystickReplay = null;
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
            Logger.recordOutput("Robot/BatteryVoltage",    RobotController.getBatteryVoltage());
            Logger.recordOutput("Robot/BrownedOut",        RobotController.isBrownedOut());
            Logger.recordOutput("Robot/CANBusUtilization", RobotController.getCANStatus().percentBusUtilization);
        }

        m_robotContainer.updateGameData();
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