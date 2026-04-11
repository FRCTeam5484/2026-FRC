package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;

public class subLimelight extends SubsystemBase {
  private boolean UseFrontLimelight = true;
  private boolean UseBackLimelight = true;
  subDrive drive;
  public subLimelight(subDrive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    sendDashboard();
    //updateFrontPose();        
    updateBackPose();
    SmartDashboard.putNumber("Targeting TY", LimelightHelpers.getTY(Constants.LimeLight.shooterTargetingName));
  }
  private void updateFrontPose(){
    if (UseFrontLimelight) {
        var driveState = drive.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.fieldPositionFrontLeft, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.fieldPositionFrontLeft);
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            drive.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
        SmartDashboard.putString("Front Limelight Pose", llMeasurement.pose.toString());
    }
    else{
        SmartDashboard.putString("Front Limelight Pose", "Disabled");
    }
  }
  private void updateBackPose(){
    if(UseBackLimelight){
        var driveState = drive.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        
        LimelightHelpers.SetRobotOrientation(Constants.LimeLight.fieldPositionBackRight, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLight.fieldPositionBackRight);
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
            drive.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
        SmartDashboard.putString("Back Limelight Pose", llMeasurement.pose.toString());
    }
    else{
        SmartDashboard.putString("Back Limelight Pose", "Disabled");
    }
  }
  public void sendDashboard(){
    SmartDashboard.putBoolean("Front Limelight", UseFrontLimelight);
    SmartDashboard.putBoolean("Back Limelight", UseBackLimelight);
    SmartDashboard.putBoolean("Shooter Has Target", shooterHasTarget());
    SmartDashboard.putBoolean("Front Has Target", frontHasTarget());
    SmartDashboard.putBoolean("Back Has Target", backHasTarget());
  }
  public boolean isFrontEnabled(){
    return UseFrontLimelight;
  } 
  public boolean isBackEnabled(){
    return UseBackLimelight;
  }
  public void toggleFront(){
    UseFrontLimelight = !UseFrontLimelight;
  }
  public void toggleBack(){
    UseBackLimelight = !UseBackLimelight;
  }
  public void disablePose(){
    UseBackLimelight = false;
    UseFrontLimelight = false;
  }
  public void enablePose(){
    UseBackLimelight = true;
    UseFrontLimelight = true;
  }
  public void setMode1(){
    LimelightHelpers.SetIMUMode(Constants.LimeLight.fieldPositionBackRight, 1);
    LimelightHelpers.SetIMUMode(Constants.LimeLight.fieldPositionFrontLeft, 1);
  }
  public void setMode4(){
    LimelightHelpers.SetIMUMode(Constants.LimeLight.fieldPositionBackRight, 4);
    LimelightHelpers.SetIMUMode(Constants.LimeLight.fieldPositionFrontLeft, 4);
  }
  public boolean frontHasTarget(){
    return LimelightHelpers.getTV(Constants.LimeLight.fieldPositionFrontLeft);
  }
  public boolean backHasTarget(){
    return LimelightHelpers.getTV(Constants.LimeLight.fieldPositionBackRight);
  }
  public boolean shooterHasTarget(){
    return LimelightHelpers.getTV(Constants.LimeLight.shooterTargetingName);
  }
}