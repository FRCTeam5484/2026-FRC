package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subHood extends SubsystemBase {
  public boolean hoodOnTarget = false;
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_hoodMotor = new TalonFX(Constants.Hood.motorId, canbus); 
  private final PositionVoltage m_hoodPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();
  private final CANcoder m_cancoder = new CANcoder(Constants.Hood.canEncoderId, canbus);
  
  public subHood() {
    ConfigureHood();
    var toApply = new CANcoderConfiguration();
    m_cancoder.getConfigurator().apply(toApply);
    BaseStatusSignal.setUpdateFrequencyForAll(100, m_cancoder.getPosition(), m_cancoder.getVelocity());
  }

  @Override
  public void periodic() {
    hoodOnTarget = m_hoodMotor.getClosedLoopError().isNear(EncoderValue(),0.1);
    SmartDashboard.putBoolean("Hood On Target", hoodOnTarget);
    SmartDashboard.putNumber("Hood Encoder", EncoderValue());
  }

  private void ConfigureHood(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    configs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    // Retry config apply up to 5 times, report if failure 
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_hoodMotor.getConfigurator().apply(configs);
      m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
      m_cancoder.setPosition(0);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // Make sure we start at 0
    m_hoodMotor.setPosition(0);
    
  }

  public void TeleOp(double joystickValue) {
    double value = Math.abs(joystickValue) <= 0.02 ? 0 : joystickValue;
    if(joystickValue > 0 && EncoderValue() > -0.01){
      Stop();
    }
    else if(joystickValue < 0 && EncoderValue() < -0.4){
      Stop();
    }
    else{
      m_hoodMotor.set(value);
    }
  }

  public void setPosition(double position)
  {
    double desiredRotations = position * 10; // Go for plus/minus 10 rotations
    if (Math.abs(desiredRotations) <= 0.1) { // Joystick deadzone
      desiredRotations = 0;
    }

    if(desiredRotations == 0){
      m_hoodMotor.setControl(m_brake);
    }
    else{
      m_hoodMotor.setControl(m_hoodPositionVoltage.withPosition(desiredRotations));
    }
  }

  public double EncoderValue(){
    return m_cancoder.getPosition().getValueAsDouble();
  }

  public void Stop() {
    m_hoodMotor.stopMotor();
    m_hoodMotor.setControl(m_brake);
  }
}