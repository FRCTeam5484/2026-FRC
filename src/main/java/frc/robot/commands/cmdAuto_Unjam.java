package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subShooter;

public class cmdAuto_Unjam extends Command {
  subBed bed;
  subFeeder feeder;
  subShooter shooter;
  public cmdAuto_Unjam(subBed bed, subFeeder feeder, subShooter shooter) {
    this.bed = bed;
    this.feeder = feeder;
    this.shooter = shooter;
    addRequirements(this.bed, this.feeder, this.shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    bed.TeleOp(-1);
    feeder.TeleOp(-1);
    shooter.TeleOp(-1);
  }

  @Override
  public void end(boolean interrupted) {
    bed.Stop();
    feeder.Stop();
    shooter.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
