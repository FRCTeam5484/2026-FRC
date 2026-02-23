package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subFeeder;

public class cmdFeeder_TeleOp extends Command {
  subFeeder m_feeder;
  DoubleSupplier m_speed;
  public cmdFeeder_TeleOp(subFeeder feeder, DoubleSupplier speed) {
    m_feeder = feeder;
    m_speed = speed;
    addRequirements(m_feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_feeder.TeleOp(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_feeder.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
