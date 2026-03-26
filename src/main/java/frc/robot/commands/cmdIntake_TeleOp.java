package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subIntake;

public class cmdIntake_TeleOp extends Command {
  subIntake intake;
  DoubleSupplier speed;
  public cmdIntake_TeleOp(subIntake intake, DoubleSupplier speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.TeleOp(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    intake.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
