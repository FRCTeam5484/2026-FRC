package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Hood;
import frc.robot.Constants.LimeLight;
import frc.robot.classes.LimelightHelpers;

public class subHood extends SubsystemBase {
  public boolean hoodOnTarget = false;
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_hoodMotor = new TalonFX(Hood.motorId, canbus); 
  private final PositionVoltage m_hoodPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();
    
  public subHood() {
    ConfigureHood();
  }

  @Override
  public void periodic() {
    hoodOnTarget = m_hoodMotor.getClosedLoopError().isNear(m_hoodMotor.getPosition().getValueAsDouble(),0.1);
    SmartDashboard.putBoolean("Hood On Target", hoodOnTarget);
    SmartDashboard.putNumber("Hood Encoder", m_hoodMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood Command", hoodPositionCommand());
  }

  private void ConfigureHood(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.5; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    configs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    // Retry config apply up to 5 times, report if failure 
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_hoodMotor.getConfigurator().apply(configs);
      m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // Make sure we start at 0
    m_hoodMotor.setPosition(0);
    
  }

  public void TeleOp(double joystickValue) {
    if(joystickValue < 0 && m_hoodMotor.getPosition().getValueAsDouble() <= 0){
      Stop();
    }
    else if(joystickValue > 0 && m_hoodMotor.getPosition().getValueAsDouble() >= 0.6){
      Stop();
    }
    else{
      m_hoodMotor.set(joystickValue);
    }
  }
  public void TeleOpNoSafe(double joystickValue) {
    double value = Math.abs(joystickValue) <= 0.02 ? 0 : joystickValue;
    m_hoodMotor.set(value);
  }

  public void setPosition()
  {
    m_hoodMotor.setControl(m_hoodPositionVoltage.withPosition(hoodPositionCommand()));
  }
  public void setPosition(double position)
  {
    m_hoodMotor.setControl(m_hoodPositionVoltage.withPosition(position));
  }

  public void ResetEncoder(){
    m_hoodMotor.setPosition(0);
  }

  public void Stop() {
    m_hoodMotor.stopMotor();
    m_hoodMotor.setControl(m_brake);
  }

  public void putHoodDown(){
    setPosition(Constants.Hood.bottomPosition);
  }

  public double hoodPositionCommand()
  {
    double distance = LimelightHelpers.getTY(LimeLight.shooterTargetingName);
    distance = Math.max(Constants.Hood.bottomPosition, Math.min(Constants.Hood.topPosition, distance));
    double normalized = (distance - Constants.Hood.bottomPosition) / (Constants.Hood.topPosition - Constants.Hood.bottomPosition);
    return Constants.Hood.topPosition + normalized * (Constants.Hood.bottomPosition - Constants.Hood.topPosition);
  }
}