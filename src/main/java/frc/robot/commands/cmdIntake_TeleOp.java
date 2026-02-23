package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subIntake;

public class cmdIntake_TeleOp extends Command {
  subIntake m_intake;
  DoubleSupplier m_speed;
  public cmdIntake_TeleOp(subIntake intake, DoubleSupplier speed) {
    m_intake = intake;
    m_speed = speed;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.TeleOp(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
