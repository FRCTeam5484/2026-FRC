package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subHopper;

public class cmdHopper_TeleOp extends Command {
  subHopper m_hopper;
  DoubleSupplier m_speed;
  public cmdHopper_TeleOp(subHopper hopper, DoubleSupplier speed) {
    m_hopper = hopper;
    m_speed = speed;
    addRequirements(m_hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_hopper.TeleOp(m_speed.getAsDouble());
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
