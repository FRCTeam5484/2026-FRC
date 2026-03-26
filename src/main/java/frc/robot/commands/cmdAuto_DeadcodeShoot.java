package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subIntake;
import frc.robot.subsystems.subShooter;

public class cmdAuto_DeadcodeShoot extends Command {
  subBed bed;
  subFeeder feeder;
  subShooter shooter;
  subIntake intake;
  
  Double m_power;
  Timer timer = new Timer();
  public cmdAuto_DeadcodeShoot(subBed bed, subFeeder feeder, subShooter shooter, subIntake intake, Double power) {
    this.bed = bed;
    this.feeder = feeder;
    this.shooter = shooter;
    this.intake = intake;
  
    m_power = power;
    addRequirements(this.bed, this.feeder, this.shooter, this.intake);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooter.TeleOp(m_power);
  }

  @Override
  public void execute() {
    if(timer.get() > 2){
      intake.TeleOp(.5);
      bed.TeleOp(1);
      feeder.TeleOp(1);
     
    }
  }

  @Override
  public void end(boolean interrupted) {
    bed.Stop();
    feeder.Stop();
    shooter.Stop();
    intake.Stop();
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
