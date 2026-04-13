package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subShooter;

public class cmdShooter_TestRPM extends Command {
  subShooter shooter;
  public cmdShooter_TestRPM(subShooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.setShooterDashboardRPM();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
