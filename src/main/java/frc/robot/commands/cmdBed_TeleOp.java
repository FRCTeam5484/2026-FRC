package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBed;

public class cmdBed_TeleOp extends Command {
  subBed bed;
  DoubleSupplier speed;
  public cmdBed_TeleOp(subBed bed, DoubleSupplier speed) {
    this.bed = bed;
    this.speed = speed;
    addRequirements(this.bed);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    bed.TeleOp(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    bed.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
