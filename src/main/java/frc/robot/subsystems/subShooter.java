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
  public boolean turretOnTarget = false;
  public boolean angleOnTarget = false;
  public double shooterRPM = 0;
  public double turretPosition = 0;
  public double anglePosition = 0;
  private final CANBus canbus = new CANBus("canivore");
  private final TalonFX m_leftLaunchMotor = new TalonFX(Constants.Shooter.leftLaunchMotorId, canbus);
  private final TalonFX m_rightLaunchMotor = new TalonFX(Constants.Shooter.rightLaunchMotorId, canbus);  
  private final TalonFX m_angleMotor = new TalonFX(Constants.Shooter.hoodMotorId, canbus); 
  //private final TalonFX m_turretMotor = new TalonFX(Constants.Shooter.turretMotorId, canbus); 
  private final TalonFX m_feederMotor = new TalonFX(Constants.Shooter.feederMotorId, canbus); 
  private final VelocityVoltage m_shooterVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage m_turretPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage m_anglePositionVoltage = new PositionVoltage(0).withSlot(0);
  private final VelocityVoltage m_feederVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();
  
  public subShooter() {
    ConfigureShooter();
    //ConfigureTurret();
    ConfigureAngle();
    ConfigureFeeder();
  }

  @Override
  public void periodic() {
    isShooterAtSpeed();
    isAngleOnTarget();
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
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(11)).withPeakReverseVoltage(Volts.of(-11));

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
  private void ConfigureAngle(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    // Retry config apply up to 5 times, report if failure 
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_angleMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // Make sure we start at 0
    m_angleMotor.setPosition(0);
  }
  private void ConfigureFeeder(){
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(11)).withPeakReverseVoltage(Volts.of(-11));

     // Retry config apply up to 5 times, report if failure
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_feederMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }
  
  public void ShooterTeleOp(double joystickValue) {
    m_leftLaunchMotor.set(Math.abs(joystickValue) <= 0.1 ? 0 : joystickValue);
  }
  public void TurretTeleOp(double joystickValue) {
    //m_turretMotor.set(Math.abs(joystickValue) <= 0.1 ? 0 : joystickValue);
  }
  public void AngleTeleOp(double joystickValue) {
    m_angleMotor.set(Math.abs(joystickValue) <= 0.02 ? 0 : joystickValue);
  }
  public void FeederTeleOp(double joystickValue) {
    m_feederMotor.set(Math.abs(joystickValue) <= 0.1 ? 0 : joystickValue);
  }

  public void setShooterRPM(double rps) {
    double desiredRotationsPerSecond = Math.abs(rps) < 0.01 ? 0 : rps * 90;
    m_leftLaunchMotor.setControl(m_shooterVelocityVoltage.withVelocity(desiredRotationsPerSecond));
  }
  public void setTurretPosition(double position) {
    double desiredRotations = position * 10; // Go for plus/minus 10 rotations
    if (Math.abs(desiredRotations) <= 0.1) { // Joystick deadzone
      desiredRotations = 0;
    }

    if(desiredRotations == 0){
      //m_turretMotor.setControl(m_brake);
    }
    else{
      //m_turretMotor.setControl(m_turretPositionVoltage.withPosition(desiredRotations));
    }
  }
  public void setAnglePosition(double position)
  {
    double desiredRotations = position * 10; // Go for plus/minus 10 rotations
    if (Math.abs(desiredRotations) <= 0.1) { // Joystick deadzone
      desiredRotations = 0;
    }

    if(desiredRotations == 0){
      m_angleMotor.setControl(m_brake);
    }
    else{
      m_angleMotor.setControl(m_anglePositionVoltage.withPosition(desiredRotations));
    }
  }
  public void setFeederRPM(double rps) {
    double desiredRotationsPerSecond = Math.abs(rps) < 0.1 ? 0 : rps * 90;
    m_feederMotor.setControl(m_feederVelocityVoltage.withVelocity(desiredRotationsPerSecond));
  }
  
  public void isShooterAtSpeed() {
    shooterAtSpeed = m_leftLaunchMotor.getClosedLoopError().isNear(shooterRPM, 1.0);
  }
  public void isTurretOnTarget() {
    //turretOnTarget = m_turretMotor.getClosedLoopError().isNear(turretPosition,1.0);
  }
  public void isAngleOnTarget() {
    angleOnTarget = m_angleMotor.getClosedLoopError().isNear(anglePosition,1.0);
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

  public void stopShooter() {
    m_leftLaunchMotor.stopMotor();
  }
  public void stopTurret() {
    //m_turretMotor.stopMotor();
  }
  public void stopAngle() {
    m_angleMotor.stopMotor();
  }
  public void stopFeeder(){
    m_feederMotor.stopMotor();
  }
}