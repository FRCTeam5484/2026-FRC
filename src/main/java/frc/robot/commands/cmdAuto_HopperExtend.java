package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subHopper;

public class cmdAuto_HopperExtend extends Command {
  subHopper m_hopper;
  double m_position;
  public cmdAuto_HopperExtend(subHopper hopper, double position) {
    m_hopper = hopper;
    m_position = position;
    addRequirements(m_hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_hopper.SetHopperPosition(m_position);
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
