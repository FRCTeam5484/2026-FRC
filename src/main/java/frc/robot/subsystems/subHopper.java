package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subHopper extends SubsystemBase {
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_hopperMotor = new TalonFX(Constants.Hopper.motorId, canbus);
  //private final CANcoder m_cancoder = new CANcoder(Constants.Climb.canCoderId, canbus);
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();
  double ClosedPosition = -1.5;
  double OpenPosition = -8.5;
  
  public subHopper() {
    configureHopper();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Hopper Encoder", m_cancoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hopper Encoder", m_hopperMotor.getPosition().getValueAsDouble());
  }

  private void configureHopper(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 1; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.05; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

     // Retry config apply up to 5 times, report if failure
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_hopperMotor.getConfigurator().apply(configs);
      m_hopperMotor.setNeutralMode(NeutralModeValue.Brake);
      m_hopperMotor.setPosition(0);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    
  }
  public void TeleOp(double value){
    m_hopperMotor.set(value);
  }
  public void Stop(){
    m_hopperMotor.stopMotor();
    m_hopperMotor.setControl(m_brake);
  }
  public void ExtendHopper(){
    SetHopperPosition(OpenPosition);
  }
  public void RetractHopper(){
    SetHopperPosition(ClosedPosition);
  }
  public void SetHopperPosition(double position){
    m_hopperMotor.setControl(m_positionVoltage.withPosition(position));
  }
  public void ResetEncoder(){
    m_hopperMotor.setPosition(0);
  }
}
