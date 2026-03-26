package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subFeeder;

public class cmdFeeder_TeleOp extends Command {
  subFeeder feeder;
  DoubleSupplier speed;
  public cmdFeeder_TeleOp(subFeeder feeder, DoubleSupplier speed) {
    this.feeder = feeder;
    this.speed = speed;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    feeder.TeleOp(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    feeder.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
