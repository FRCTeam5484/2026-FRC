package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subHopper;
import frc.robot.subsystems.subIntake;
import frc.robot.subsystems.subShooter;

public class cmdAuto_AutoShoot extends Command {
  subBed m_bed;
  subFeeder m_feeder;
  subShooter m_shooter;
  subIntake m_intake;
  subHopper m_hopper;
  Timer timer = new Timer();
  public cmdAuto_AutoShoot(subBed bed, subFeeder feeder, subShooter shooter, subIntake intake, subHopper hopper) {
    m_bed = bed;
    m_feeder = feeder;
    m_shooter = shooter;
    m_intake = intake;
    m_hopper = hopper;
    addRequirements(m_bed, m_feeder, m_shooter, m_intake, m_hopper);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    m_shooter.TeleOp(1);
  }

  @Override
  public void execute() {
    if(timer.get() > 1){
      m_intake.TeleOp(.5);
      m_bed.TeleOp(1);
      m_feeder.TeleOp(1);
      if(timer.get() > 1.5){
        m_hopper.SetHopperPosition(-1.5);
      }
      else if ( timer.get() > 2){
        m_hopper.SetHopperPosition(-4);
      }
      else if ( timer.get() > 2.5){
        m_hopper.SetHopperPosition(-1.5);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_bed.Stop();
    m_feeder.Stop();
    m_shooter.Stop();
    m_intake.Stop();
    m_hopper.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
