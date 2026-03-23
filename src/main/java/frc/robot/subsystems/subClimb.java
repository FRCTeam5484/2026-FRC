package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subClimb extends SubsystemBase {
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_climbMotor = new TalonFX(Constants.Climb.motorId, canbus);
  //private final CANcoder m_cancoder = new CANcoder(Constants.Climb.canCoderId, canbus);
  DigitalInput m_toplimitSwitch = new DigitalInput(Constants.Climb.topLimitSwitchId);
  DigitalInput m_bottomlimitSwitch = new DigitalInput(Constants.Climb.bottomLimitSwitchId);
  private final NeutralOut m_brake = new NeutralOut();

  public subClimb() {
    configureClimb();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Climb Encoder", m_cancoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climb Encoder", m_climbMotor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Climb Top Limit", m_toplimitSwitch.get());
    SmartDashboard.putBoolean("Climb Bottom Limit", m_bottomlimitSwitch.get());
    if(m_bottomlimitSwitch.get()) m_climbMotor.setPosition(0);
  }
  private void configureClimb(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.6; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(11)).withPeakReverseVoltage(Volts.of(-11));
    configs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

     // Retry config apply up to 5 times, report if failure
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_climbMotor.getConfigurator().apply(configs);
      m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
      m_climbMotor.setPosition(0);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public void TeleOp(double value){
    if(value > 0 && !m_toplimitSwitch.get())
    {
      Stop();
    }
    else if (value < 0 && !m_bottomlimitSwitch.get())
    {
      Stop();
    }
    else if(value == 0)
    {
      Stop();
    }
    else
    {
      m_climbMotor.set(value);
    }
  }
  public void Stop(){
    m_climbMotor.stopMotor();
    m_climbMotor.setControl(m_brake);
  }
  public void RaiseClimb(){
    if(!m_toplimitSwitch.get())
    {
      Stop();
    }
    else
    {
      m_climbMotor.set(1);
      //m_climbMotor.setControl(m_positionVoltage.withPosition(-85));
    }
  }
  public void LowerClimb(){
    if(!m_bottomlimitSwitch.get())
    {
      Stop();
    }
    else
    {
      m_climbMotor.set(-1);
      //m_climbMotor.setControl(m_positionVoltage.withPosition(0));
    }
  }
}