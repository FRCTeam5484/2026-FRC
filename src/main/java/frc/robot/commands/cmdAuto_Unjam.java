package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subShooter;

public class cmdAuto_Unjam extends Command {
  subBed m_bed;
  subFeeder m_feeder;
  subShooter m_shooter;
  public cmdAuto_Unjam(subBed bed, subFeeder feeder, subShooter shooter) {
    m_bed = bed;
    m_feeder = feeder;
    m_shooter = shooter;
    addRequirements(m_bed, m_feeder, m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_bed.TeleOp(-1);
    m_feeder.TeleOp(-1);
    m_shooter.TeleOp(-1);
  }

  @Override
  public void end(boolean interrupted) {
    m_bed.Stop();
    m_feeder.Stop();
    m_shooter.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
