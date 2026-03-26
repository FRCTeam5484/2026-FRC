package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subHopper;

public class cmdHopper_Extend extends Command {
  subHopper hopper;
  public cmdHopper_Extend(subHopper hopper) {
    this.hopper = hopper;
    addRequirements(this.hopper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hopper.ExtendHopper();
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
