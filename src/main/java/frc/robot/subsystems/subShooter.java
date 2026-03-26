package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class subShooter extends SubsystemBase {
  public String gameData;
  public String activeHub;
  public boolean myHubActive = false;
  public boolean shooterAtSpeed = false;
  public double shooterRPM = 0;
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_leftLaunchMotor = new TalonFX(Constants.Shooter.leftMotorId, canbus);
  private final TalonFX m_rightLaunchMotor = new TalonFX(Constants.Shooter.rightMotorId, canbus);  
  private final VelocityVoltage m_shooterVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  
  
  public subShooter() {
    ConfigureShooter();
  }

  @Override
  public void periodic() {
    isShooterAtSpeed();
    SmartDashboard.putNumber("Shooter Power Command", CommandPower());
    SmartDashboard.putNumber("Shooter RPS Command", CommandRPM()/60);
    SmartDashboard.putNumber("Shooter RPS", m_leftLaunchMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter At Speed", shooterAtSpeed);
    
  }

  private void ConfigureShooter(){
    BaseStatusSignal.setUpdateFrequencyForAll(200, m_leftLaunchMotor.getVelocity());
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 1.5; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    configs.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(0));
    configs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

     /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_leftLaunchMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    m_rightLaunchMotor.setControl(new Follower(m_leftLaunchMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }
  
  
  public void TeleOp(double speed) {
    m_leftLaunchMotor.set(Math.abs(speed) <= 0.1 ? 0 : speed);
  }
  
  public void setShooterRPM() {
    m_leftLaunchMotor.setControl(m_shooterVelocityVoltage.withVelocity(CommandRPM() / 60));
  }

  public void setShooterPower() {
    m_leftLaunchMotor.set(CommandPower());
  }
  
  public void isShooterAtSpeed() {
    shooterAtSpeed = m_leftLaunchMotor.getClosedLoopError().isNear(shooterRPM, 1.0);
  }

  public void Stop() {
    m_leftLaunchMotor.stopMotor();
  }

  public double CommandPower()
  {  
    if(LimelightHelpers.getTV(Constants.LimeLight.shooterTargetingName))
    {  
      double MaxPower = 1;
      double MinPower = 0.6;
      double distance = LimelightHelpers.getTY(Constants.LimeLight.shooterTargetingName);
      distance = Math.max(Constants.TargetingDistance.minDistance, Math.min(Constants.TargetingDistance.maxDistance, distance));
      double normalized = (distance - Constants.TargetingDistance.minDistance) / (Constants.TargetingDistance.maxDistance - Constants.TargetingDistance.minDistance);
      return MaxPower + normalized * (MinPower - MaxPower);
    }
    else
    {
      return 0;
    }
  }

  public double CommandRPM()
  {    
    if(LimelightHelpers.getTV(Constants.LimeLight.shooterTargetingName))
    {
      double MaxRPM = 5500;
      double MinRPM = 3500;
      double distance = LimelightHelpers.getTY(Constants.LimeLight.shooterTargetingName);
      distance = Math.max(Constants.TargetingDistance.minDistance, Math.min(Constants.TargetingDistance.maxDistance, distance));
      double normalized = (distance - Constants.TargetingDistance.minDistance) / (Constants.TargetingDistance.maxDistance - Constants.TargetingDistance.minDistance);
      return MaxRPM + normalized * (MinRPM - MaxRPM);
    }
    else
    {
      return 0;
    }
  }
}