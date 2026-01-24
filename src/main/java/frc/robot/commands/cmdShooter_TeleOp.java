package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subShooter;

public class cmdShooter_TeleOp extends Command {
  subShooter m_shooter = new subShooter();
  DoubleSupplier m_stickValue;
  public cmdShooter_TeleOp(subShooter shooter, DoubleSupplier stickValue) {
    m_shooter = shooter;
    m_stickValue = stickValue;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.runShooter(m_stickValue.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
