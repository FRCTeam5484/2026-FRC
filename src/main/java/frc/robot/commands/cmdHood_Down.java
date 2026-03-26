package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subHood;

public class cmdAuto_HoodDown extends Command {
  subHood hood;
  Timer time = new Timer();
  public cmdAuto_HoodDown(subHood hood) {
    this.hood = hood;
    addRequirements(this.hood);
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    hood.putHoodDown();
  }

  @Override
  public void end(boolean interrupted) {
    hood.Stop();
  }

  @Override
  public boolean isFinished() {
    return time.get() > 1.5;
  }
}
