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
  private final SwerveRequest.RobotCentric kdrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  subDrive drive;
  subHood hood;
  subShooter shooter;
  subBed bed;
  subFeeder feeder;
  subIntake intake;
  DoubleSupplier moveCommand;

  Timer timer = new Timer();

  public cmdAuto_AutoAlignShootMove(subDrive drive, subHood hood, subShooter shooter, subBed bed, subFeeder feeder, subIntake intake, DoubleSupplier moveCommand) {
    this.drive = drive;
    this.hood = hood;
    this.shooter = shooter;
    this.bed = bed;
    this.feeder = feeder;
    this.intake = intake;
    this.moveCommand = moveCommand;
    addRequirements(this.drive, this.hood, this.shooter, this.bed, this.feeder, this.intake);
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
      drive.applyRequest(() -> kdrive.withVelocityX(moveCommand.getAsDouble()).withVelocityY(0).withRotationalRate(turnCommand())).execute();
      hood.setPosition();
      //shooter.setShooterPower();
      shooter.setShooterRPM();
      if(timer.get() > 0.3){
        intake.TeleOp(.5);
        feeder.TeleOp(1);
        bed.TeleOp(.7);
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
    intake.Stop();
    drive.applyRequest(() -> kdrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
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