package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import frc.robot.classes.TunerConstants;
import frc.robot.subsystems.subDrive;
import frc.robot.subsystems.subHood;
import frc.robot.subsystems.subShooter;

public class cmdAuto_AutoAlign extends Command {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  subDrive m_drive;
  subHood m_hood;
  subShooter m_shooter;
  public cmdAuto_AutoAlign(subDrive drive, subHood hood, subShooter shooter) {
    m_drive = drive;
    m_hood = hood;
    m_shooter = shooter;
    addRequirements(m_drive, m_hood, m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(hasTarget())
    {
      //m_drive.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(turnCommand()));
      //m_hood.setPosition(hoodCommand());
      //m_shooter.TeleOp(shooterCommand());
      System.out.println("Turn Command: " + String.valueOf(turnCommand()) + " Hood Command: " + String.valueOf(hoodCommand()) + " Shooter Command: " + shooterCommand());
    }
    else
    {
      System.out.println("!! NO TARGET !!");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_hood.Stop();
    m_shooter.Stop();
    m_drive.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double turnCommand()
  {
    double kP = .035;
    double targetingAngularVelocity = LimelightHelpers.getTX(Constants.LimeLight.shooterTargetingName) * kP;
    targetingAngularVelocity *= MaxAngularRate;
    targetingAngularVelocity *= -1.0;
    return targetingAngularVelocity;
  }
  private double hoodCommand()
  {    
    double MIN_INPUT = -20.0;
    double MAX_INPUT = 20.0;
    double MIN_OUTPUT = -0.01;
    double MAX_OUTPUT = -0.4;
    double distance = LimelightHelpers.getTY(Constants.LimeLight.shooterTargetingName);
    distance = Math.max(MIN_INPUT, Math.min(MAX_INPUT, distance));
    double normalized = (distance - MIN_INPUT) / (MAX_INPUT - MIN_INPUT);
    return MIN_OUTPUT + normalized * (MAX_OUTPUT - MIN_OUTPUT);
  }
  private double shooterCommand()
  {    
    double MIN_INPUT = -20.0;
    double MAX_INPUT = 20.0;
    double MIN_OUTPUT = .6;
    double MAX_OUTPUT = 1;
    double distance = LimelightHelpers.getTY(Constants.LimeLight.shooterTargetingName);
    distance = Math.max(MIN_INPUT, Math.min(MAX_INPUT, distance));
    double normalized = (distance - MIN_INPUT) / (MAX_INPUT - MIN_INPUT);
    return MIN_OUTPUT + normalized * (MAX_OUTPUT - MIN_OUTPUT);
  }
  private boolean hasTarget()
  {
    return LimelightHelpers.getTV(Constants.LimeLight.shooterTargetingName);
  }
}