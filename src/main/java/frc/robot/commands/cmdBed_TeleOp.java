package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subBed;

public class cmdBed_TeleOp extends Command {
  subBed m_bed;
  DoubleSupplier m_speed;
  public cmdBed_TeleOp(subBed bed, DoubleSupplier speed) {
    m_bed = bed;
    m_speed = speed;
    addRequirements(m_bed);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_bed.TeleOp(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_bed.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
