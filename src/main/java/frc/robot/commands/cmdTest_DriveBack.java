package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subDrive;

public class cmdTest_DriveBack extends Command {
  subDrive drivetrain;
  SwerveRequest.FieldCentric drive;
  Timer time = new Timer();
  public cmdTest_DriveBack(subDrive drivetrain, SwerveRequest.FieldCentric drive) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    drivetrain.applyRequest(() ->
    drive.withVelocityX(0.2) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() ->
    drive.withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return time.get() > 1 ? true : false;
  }
}
