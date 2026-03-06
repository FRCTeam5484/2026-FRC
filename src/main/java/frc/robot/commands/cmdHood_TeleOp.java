package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subHood;

public class cmdHood_TeleOp extends Command {
  subHood m_hood;
  DoubleSupplier m_speed;
  public cmdHood_TeleOp(subHood hood, DoubleSupplier speed) {
    m_hood = hood;
    m_speed = speed;
    addRequirements(m_hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_hood.TeleOp(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_hood.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
