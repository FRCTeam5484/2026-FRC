package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subClimb;

public class cmdClimb_Lower extends Command {
  subClimb climb;
  public cmdClimb_Lower(subClimb climb) {
    this.climb = climb;
    addRequirements(this.climb);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climb.LowerClimb();
  }

  @Override
  public void end(boolean interrupted) {
    climb.Stop();}

  @Override
  public boolean isFinished() {
    return !climb.m_bottomlimitSwitch.get();
  }
}
