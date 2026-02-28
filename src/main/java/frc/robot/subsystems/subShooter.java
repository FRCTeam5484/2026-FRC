package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class subShooter extends SubsystemBase {
  public String gameData;
  public String activeHub;
  public boolean myHubActive = false;
  public boolean shooterAtSpeed = false;
  public double shooterRPM = 0;
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_leftLaunchMotor = new TalonFX(Constants.Shooter.leftLaunchMotorId, canbus);
  private final TalonFX m_rightLaunchMotor = new TalonFX(Constants.Shooter.rightLaunchMotorId, canbus);  
  private final VelocityVoltage m_shooterVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  
  public subShooter() {
    ConfigureShooter();
  }

  @Override
  public void periodic() {
    isShooterAtSpeed();
    checkHubStatus();
  }

  private void ConfigureShooter(){
    BaseStatusSignal.setUpdateFrequencyForAll(200, m_leftLaunchMotor.getVelocity());
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    configs.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

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
  
  
  public void TeleOp(double joystickValue) {
    m_leftLaunchMotor.set(Math.abs(joystickValue) <= 0.1 ? 0 : joystickValue);
  }
  
  public void setShooterRPM(double rps) {
    m_leftLaunchMotor.setControl(m_shooterVelocityVoltage.withVelocity(rps * 90));
  }
  
  public void isShooterAtSpeed() {
    shooterAtSpeed = m_leftLaunchMotor.getClosedLoopError().isNear(shooterRPM, 1.0);
  }
  
  public void checkHubStatus() {
    if(DriverStation.isFMSAttached() && DriverStation.isEnabled()){
      gameData = DriverStation.getGameSpecificMessage();
      if(gameData.length() > 0)
      {
        switch (gameData.charAt(0))
        {
          case 'B' :
            activeHub = "Blue";
            myHubActive = DriverStation.getAlliance().get().equals(Alliance.Blue) ? true : false;
            break;
          case 'R' :
            activeHub = "Red";
            myHubActive = DriverStation.getAlliance().get().equals(Alliance.Red) ? true : false;
            break;
          default :
            activeHub = "Look at field";
            myHubActive = false;
            break;
        }
      }
      else{
        activeHub = "NA";
        myHubActive = false;
      }
    }
  }

  public void Stop() {
    m_leftLaunchMotor.stopMotor();
  }
}