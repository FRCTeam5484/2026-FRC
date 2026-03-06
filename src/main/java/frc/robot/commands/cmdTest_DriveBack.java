package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subDrive;

public class cmdTest_DriveBack extends Command {
  subDrive drivetrain;
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  Timer time = new Timer();
  
  public cmdTest_DriveBack(subDrive drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0));
  }

  @Override
  public boolean isFinished() {
    return time.get() > 1 ? true : false;
  }
}
