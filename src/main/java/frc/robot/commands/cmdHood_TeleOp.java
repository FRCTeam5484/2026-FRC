package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subHood;

public class cmdHood_TeleOp extends Command {
  subHood hood;
  DoubleSupplier speed;
  public cmdHood_TeleOp(subHood hood, DoubleSupplier speed) {
    this.hood = hood;
    this.speed = speed;
    addRequirements(this.hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hood.TeleOpNoSafe(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    hood.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
