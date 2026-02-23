package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subTurret extends SubsystemBase {
  public boolean turretOnTarget = false;
  public double turretPosition = 0;
  private final CANBus canbus = new CANBus("canivore");
  private final TalonFX m_turretMotor = new TalonFX(Constants.Shooter.turretMotorId, canbus); 
  private final PositionVoltage m_turretPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();
  //private final CANcoder m_cancoder = new CANcoder(Constants.Shooter.turretEncoder, canbus);
  public subTurret() {
    ConfigureTurret();
  }

  @Override
  public void periodic() {
    //turretOnTarget = m_turretMotor.getClosedLoopError().isNear(turretPosition,1.0);
    //turretPosition = m_cancoder.getPosition().getValueAsDouble();
    //SmartDashboard.putBoolean("Turret On Target", turretOnTarget);
    //SmartDashboard.putNumber("Turret Encoder", turretPosition);
  }
  private void ConfigureTurret(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    // Retry config apply up to 5 times, report if failure
     
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      //status = m_turretMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // Make sure we start at 0
    //m_turretMotor.setPosition(0);
    
  }
  
  public void TeleOp(double joystickValue) {
    m_turretMotor.set(Math.abs(joystickValue) <= 0.1 ? 0 : joystickValue);
  }
  public void setPosition(double position) {
    double desiredRotations = position * 10; // Go for plus/minus 10 rotations
    if (Math.abs(desiredRotations) <= 0.1) { // Joystick deadzone
      desiredRotations = 0;
    }

    if(desiredRotations == 0){
      m_turretMotor.setControl(m_brake);
    }
    else{
      m_turretMotor.setControl(m_turretPositionVoltage.withPosition(desiredRotations));
    }
  }
  public void Stop() {
    m_turretMotor.stopMotor();
  }
}
