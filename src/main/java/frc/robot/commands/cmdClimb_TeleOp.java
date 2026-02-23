package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subClimb;

public class cmdClimb_TeleOp extends Command {
  subClimb m_climb;
  DoubleSupplier m_speed;
  public cmdClimb_TeleOp(subClimb climb, DoubleSupplier speed) {
    m_climb = climb;
    m_speed = speed;
    addRequirements(m_climb);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climb.TeleOp(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_climb.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
