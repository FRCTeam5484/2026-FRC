package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subClimb;

public class cmdAuto_ClimbRaise extends Command {
  subClimb m_climb;
  public cmdAuto_ClimbRaise(subClimb climb) {
    m_climb = climb;
    addRequirements(climb);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climb.RaiseClimb();
  }

  @Override
  public void end(boolean interrupted) {
    m_climb.Stop();
  }

  @Override
  public boolean isFinished() {
    return !m_climb.m_toplimitSwitch.get();
  }
}
