package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subTurret;

public class cmdTurret_TeleOp extends Command {
  subTurret m_turret;
  DoubleSupplier m_speed;
  public cmdTurret_TeleOp(subTurret turret, DoubleSupplier speed) {
    m_turret = turret;
    m_speed = speed;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_turret.TeleOp(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
