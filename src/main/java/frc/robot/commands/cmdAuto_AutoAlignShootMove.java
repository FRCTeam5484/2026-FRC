package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import frc.robot.classes.TunerConstants;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subDrive;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subHood;
import frc.robot.subsystems.subIntake;
import frc.robot.subsystems.subShooter;

public class cmdAuto_AutoAlignShootMove extends Command {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  subDrive m_drive;
  subHood m_hood;
  subShooter m_shooter;
  subBed m_bed;
  subFeeder m_feeder;
  subIntake m_intake;
  DoubleSupplier m_moveCommand;

  Timer timer = new Timer();

  public cmdAuto_AutoAlignShootMove(subDrive drive, subHood hood, subShooter shooter, subBed bed, subFeeder feeder, subIntake intake, DoubleSupplier moveCommand) {
    m_drive = drive;
    m_hood = hood;
    m_shooter = shooter;
    m_bed = bed;
    m_feeder = feeder;
    m_intake = intake;
    m_moveCommand = moveCommand;
    addRequirements(m_drive, m_hood, m_shooter, m_bed, m_feeder, m_intake);
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
      m_drive.applyRequest(() -> drive.withVelocityX(m_moveCommand.getAsDouble()).withVelocityY(0).withRotationalRate(turnCommand())).execute();
      m_hood.setPosition();
      //m_shooter.setShooterPower();
      m_shooter.setShooterRPM();
      if(timer.get() > 1.5){
        m_intake.TeleOp(.5);
        m_feeder.TeleOp(1);
        m_bed.TeleOp(1);
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
    m_hood.Stop();
    m_shooter.Stop();
    m_feeder.Stop();
    m_bed.Stop();
    m_intake.Stop();
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

  private boolean hasTarget()
  {
    return LimelightHelpers.getTV(Constants.LimeLight.shooterTargetingName);
  }
}