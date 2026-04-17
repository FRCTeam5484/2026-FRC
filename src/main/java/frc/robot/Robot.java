// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.classes.LimelightHelpers;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        m_robotContainer.lime.setMode1();
    }
    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent() && ally.get() == Alliance.Red) {
            m_robotContainer.drivetrain.resetRotation(Rotation2d.fromDegrees(180));
            LimelightHelpers.SetRobotOrientation(Constants.LimeLight.fieldPositionBackRight, 180, 0, 0, 0, 0, 0);
        }
        m_robotContainer.lime.enablePose();
        m_robotContainer.lime.setMode4();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        m_robotContainer.lime.disablePose();
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
    public void simulationPeriodic() {}
}
