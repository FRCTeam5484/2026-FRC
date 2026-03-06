// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.classes.LimelightHelpers;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    private ShuffleboardTab tabLimelight = Shuffleboard.getTab("LimeLight");
    private GenericEntry frontLimelightEnabled = tabLimelight.add("Front Limelight", true).getEntry();
    private GenericEntry backLimelightEnabled = tabLimelight.add("Back Limelight", true).getEntry();

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
        UpdateShuffleboard();
        UpdatePose();        
    }

    private void UpdateShuffleboard(){
        /// Limelight Widgets
        //frontLimelightEnabled = tabLimelight.add("Front Limelight", true).getEntry();
        //backLimelightEnabled = tabLimelight.add("Back Limelight", true).getEntry();
    }

    private void UpdatePose(){
        var driveState = m_robotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.fieldPositionFrontLeft, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate llFrontMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.fieldPositionFrontLeft);
        if (llFrontMeasurement != null && llFrontMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0 && frontLimelightEnabled.getBoolean(true)) {
            m_robotContainer.drivetrain.addVisionMeasurement(llFrontMeasurement.pose, llFrontMeasurement.timestampSeconds);
        }
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.fieldPositionBackRight, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate llBackMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.fieldPositionBackRight);
        if (llBackMeasurement != null && llBackMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0 && backLimelightEnabled.getBoolean(true)) {
            m_robotContainer.drivetrain.addVisionMeasurement(llBackMeasurement.pose, llBackMeasurement.timestampSeconds);
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
        //m_robotContainer.limelight.configureFrontLeft();
        //m_robotContainer.limelight.configureBackRight();
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
        m_robotContainer.ConfigureTeleOpControls();
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
        m_robotContainer.ConfigureTestControls();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
