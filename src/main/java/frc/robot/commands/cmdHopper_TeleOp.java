package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subHopper;

public class cmdHopper_TeleOp extends Command {
  subHopper hopper;
  DoubleSupplier speed;
  public cmdHopper_TeleOp(subHopper hopper, DoubleSupplier speed) {
    this.hopper = hopper;
    this.speed = speed;
    addRequirements(this.hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hopper.TeleOp(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    hopper.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
