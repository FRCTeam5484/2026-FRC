package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subHood;
import frc.robot.subsystems.subIntake;
import frc.robot.subsystems.subShooter;

public class cmdAuto_AutoShoot extends Command {
  subHood hood;
  subShooter shooter;
  subBed bed;
  subFeeder feeder;
  subIntake intake;
  Timer timer = new Timer();

  public cmdAuto_AutoShoot(subHood hood, subShooter shooter, subBed bed, subFeeder feeder) {
    this.hood = hood;
    this.shooter = shooter;
    this.bed = bed;
    this.feeder = feeder;
    addRequirements(this.hood, this.shooter, this.bed, this.feeder);
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {
    if(hasTarget())
    {
      timer.start();
      hood.setPosition();
      shooter.setShooterRPM();
      if(timer.get() > 1){
        feeder.TeleOp(1);
        bed.TeleOp(1);
      }
    }
    else
    {
      timer.stop();
      timer.reset();
      System.out.println("!! NO TARGET !!");
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

  private boolean hasTarget()
  {
    return LimelightHelpers.getTV(Constants.LimeLight.shooterTargetingName);
  }
}
