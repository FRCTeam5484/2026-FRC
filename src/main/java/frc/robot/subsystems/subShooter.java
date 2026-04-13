package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.LimelightHelpers;
import static edu.wpi.first.units.Units.*;

import java.util.Map;

import javax.lang.model.util.ElementScanner14;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class subShooter extends SubsystemBase {
  public String gameData;
  public String activeHub;
  public boolean myHubActive = false;
  public boolean shooterAtSpeed = false;
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_leftLaunchMotor = new TalonFX(Constants.Shooter.leftMotorId, canbus);
  private final TalonFX m_rightLaunchMotor = new TalonFX(Constants.Shooter.rightMotorId, canbus);  
  private final VelocityVoltage m_shooterVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private double currentRPMCommand = 0;
  
  public subShooter() {
    ConfigureShooter();          
  }

  @Override
  public void periodic() {
    isShooterAtSpeed();
    SmartDashboard.putNumber("Shooter RPS Command", CommandRPM()/60);
    SmartDashboard.putNumber("Shooter RPS", m_leftLaunchMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter At Speed", shooterAtSpeed);
    SmartDashboard.putNumber("Shooter RPM Command", CommandRPM());
    
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
    configs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

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
    if(currentRPMCommand != CommandRPM())
    {
      currentRPMCommand = CommandRPM();
      m_leftLaunchMotor.setControl(m_shooterVelocityVoltage.withVelocity(currentRPMCommand / 60));
    }
  }
  
  public void isShooterAtSpeed() {
    shooterAtSpeed = m_leftLaunchMotor.getClosedLoopError().isNear(m_leftLaunchMotor.getVelocity().getValueAsDouble(), 10.0);
  }

  public void Stop() {
    m_leftLaunchMotor.stopMotor();
  }

  public double CommandRPM()
  {    
    if(LimelightHelpers.getTV(Constants.LimeLight.shooterTargetingName))
    {      
      
      double distance = LimelightHelpers.getTY(Constants.LimeLight.shooterTargetingName);  
      //return MathUtil.clamp(-34.16088 * distance + 3703.9041, Constants.Shooter.MinRPM, Constants.Shooter.MaxRPM);
      if (distance > 6) // close 24 to -2
      {
        return MathUtil.clamp(-34.16088 * distance + 3703.9041, Constants.Shooter.MinRPM, Constants.Shooter.MaxRPM);
      }
      else if (distance>-3 && distance<=6) // middle -2 to -12
      {
        return MathUtil.clamp(-38.16088 * distance + 3603.9041, Constants.Shooter.MinRPM, Constants.Shooter.MaxRPM);
      }
      else if(distance<=-3) // far-12 to -20
      {
        return MathUtil.clamp(-40.16088 * distance + 3703.9041, Constants.Shooter.MinRPM, Constants.Shooter.MaxRPM);
        //return MathUtil.clamp(-63.6735 * distance + 2800.4898, Constants.Shooter.MinRPM, Constants.Shooter.MaxRPM);
      }
      else
      {
        return MathUtil.clamp(-36.16088 * distance + 3703.9041, Constants.Shooter.MinRPM, Constants.Shooter.MaxRPM);
        //return MathUtil.clamp(-65.6735 * distance + 3700.4898, Constants.Shooter.MinRPM, Constants.Shooter.MaxRPM);
      }
    }
    else
    {
      return 0;
    }
  }
}