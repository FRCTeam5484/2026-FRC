package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import frc.robot.classes.LimelightHelpers.PoseEstimate;

public class subLimeLight extends SubsystemBase {
  public subLimeLight() {
    configureFrontLeft();
    configureBackRight();
  }

  @Override
  public void periodic() {
    
  }

  public void configureFrontLeft(){
    // Set a custom crop window for improved performance (-1 to 1 for each value)
    LimelightHelpers.setCropWindow(Constants.LimeLight.fieldPositionFrontLeft, 0, 0, 0, 0);

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(Constants.LimeLight.fieldPositionFrontLeft,
        0.279,    // Forward offset (meters)
        0.279,    // Side offset (meters)
        0,    // Height offset (meters)
        0.0,    // Roll (degrees)
        15.0,   // Pitch (degrees)
        45.0     // Yaw (degrees)
    );

    // Set AprilTag offset tracking point (meters)
    //LimelightHelpers.setFiducial3DOffset(Constants.LimeLight.fieldPositionFrontLeft,
    //    0.0,    // Forward offset
    //    0.0,    // Side offset
    //    0.5     // Height offset
    //);

    // Configure AprilTag detection
    
    LimelightHelpers.SetFiducialDownscalingOverride(Constants.LimeLight.fieldPositionFrontLeft, 2.0f); // Process at half resolution

    // Adjust keystone crop window (-0.95 to 0.95 for both horizontal and vertical)
    //LimelightHelpers.setKeystone("", 0.1, -0.05);
  }
  public void configureBackRight(){
    // Set a custom crop window for improved performance (-1 to 1 for each value)
    LimelightHelpers.setCropWindow(Constants.LimeLight.fieldPositionBackRight, 0, 0, 0, 0);

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(Constants.LimeLight.fieldPositionBackRight,
        -0.279,    // Forward offset (meters)
        -0.279,    // Side offset (meters)
        0,    // Height offset (meters)
        0.0,    // Roll (degrees)
        15.0,   // Pitch (degrees)
        -135.0     // Yaw (degrees)
    );

    // Set AprilTag offset tracking point (meters)
    //LimelightHelpers.setFiducial3DOffset(Constants.LimeLight.fieldPositionBackRight,
    //    0.0,    // Forward offset
    //    0.0,    // Side offset
    //    0.5     // Height offset
    //);

    // Configure AprilTag detection
    
    LimelightHelpers.SetFiducialDownscalingOverride(Constants.LimeLight.fieldPositionBackRight, 2.0f); // Process at half resolution

    // Adjust keystone crop window (-0.95 to 0.95 for both horizontal and vertical)
    //LimelightHelpers.setKeystone("", 0, -0);
  }
}
