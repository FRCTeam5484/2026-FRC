package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subHood;
import frc.robot.subsystems.subShooter;

public class cmdAuto_RelayToAlliance extends Command {
  subHood m_hood;
  subShooter m_shooter;
  subBed m_bed;
  subFeeder m_feeder;
  Timer timer = new Timer();

  public cmdAuto_RelayToAlliance(subHood hood, subShooter shooter, subBed bed, subFeeder feeder) {
    m_hood = hood;
    m_shooter = shooter;
    m_bed = bed;
    m_feeder = feeder;
    addRequirements(m_hood, m_shooter, m_bed, m_feeder);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    m_hood.setPosition(0.6);
    m_shooter.TeleOp(0.7);
    if(timer.get() > 1.5){
      m_feeder.TeleOp(1);
      m_bed.TeleOp(1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_hood.Stop();
    m_shooter.Stop();
    m_feeder.Stop();
    m_bed.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}