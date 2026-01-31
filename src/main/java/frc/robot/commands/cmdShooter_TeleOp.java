package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subShooter;

public class cmdShooter_TeleOp extends Command {
  subShooter m_shooter = new subShooter();
  DoubleSupplier m_shooterStickValue;
  DoubleSupplier m_turretStickValue;
  DoubleSupplier m_angleStickValue;
  public cmdShooter_TeleOp(subShooter shooter, DoubleSupplier shooterStickValue, DoubleSupplier turretStickValue, DoubleSupplier angleStickValue) {
    m_shooter = shooter;
    m_shooterStickValue = shooterStickValue;
    m_turretStickValue = turretStickValue;
    m_angleStickValue = angleStickValue;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.ShooterTeleOp(m_shooterStickValue.getAsDouble());
    m_shooter.TurretTeleOp(m_turretStickValue.getAsDouble());
    m_shooter.AngleTeleOp(m_angleStickValue.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_shooter.stopTurret();
    m_shooter.stopAngle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}