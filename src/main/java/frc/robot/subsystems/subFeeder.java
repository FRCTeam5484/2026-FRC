package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subFeeder extends SubsystemBase {
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_feederMotor = new TalonFX(Constants.Feeder.motorId, canbus); 
  private final VelocityVoltage m_feederVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  public subFeeder() {
    ConfigureFeeder();
  }

  @Override
  public void periodic() {}
  private void ConfigureFeeder(){
    TalonFXConfiguration configs = new TalonFXConfiguration()
    .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(30))
                .withStatorCurrentLimitEnable(true)
        );;

    // Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(11)).withPeakReverseVoltage(Volts.of(-11));

    configs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

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
  public void TeleOp(double joystickValue) {
    m_feederMotor.set(Math.abs(joystickValue) <= 0.05 ? 0 : joystickValue);
  }
  public void setFeederRPM(double rps) {
    m_feederMotor.setControl(m_feederVelocityVoltage.withVelocity(rps * 90));
  }
  public void Stop(){
    m_feederMotor.stopMotor();
  }
}
