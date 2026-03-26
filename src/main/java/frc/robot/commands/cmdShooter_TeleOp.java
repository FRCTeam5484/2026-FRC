package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subShooter;

public class cmdShooter_TeleOp extends Command {
  subShooter shooter;
  DoubleSupplier speed;
  public cmdShooter_TeleOp(subShooter shooter, DoubleSupplier speed) {
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.TeleOp(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
