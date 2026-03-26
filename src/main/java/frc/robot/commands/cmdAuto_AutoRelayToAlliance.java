package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subHood;
import frc.robot.subsystems.subShooter;

public class cmdAuto_RelayToAlliance extends Command {
  subHood hood;
  subShooter shooter;
  subBed bed;
  subFeeder feeder;
  Timer timer = new Timer();

  public cmdAuto_RelayToAlliance(subHood hood, subShooter shooter, subBed bed, subFeeder feeder) {
    this.hood = hood;
    this.shooter = shooter;
    this.bed = bed;
    this.feeder = feeder;
    addRequirements(this.hood, this.shooter, this.bed, this.feeder);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    hood.setPosition(0.6);
    shooter.TeleOp(0.7);
    if(timer.get() > 1.5){
     feeder.TeleOp(1);
      bed.TeleOp(1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    hood.Stop();
    shooter.Stop();
   feeder.Stop();
    bed.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}