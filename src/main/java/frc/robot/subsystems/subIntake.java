
package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subIntake extends SubsystemBase {
  //SparkMax m_topIntakeMotor = new SparkMax(Constants.Intake.topMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  //SparkMax m_bottomIntakeMotor = new SparkMax(Constants.Intake.bottomMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final CANBus canbus = new CANBus("SubSystems");
  private final TalonFX m_intake = new TalonFX(Constants.Intake.x44MotorId, canbus);

  public subIntake() {
    /*
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kCoast)
      .inverted(false);
    
      m_topIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_bottomIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
      */

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
      status = m_intake.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Intake Top Motor Output", m_topIntakeMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Intake Bottom Motor Output", m_bottomIntakeMotor.getAppliedOutput());
  }

  public void TeleOp(double value){
    m_intake.set(Math.abs(value) <= 0.1 ? 0 : value);
    //m_topIntakeMotor.set(value);
    //m_bottomIntakeMotor.set(value);
  }
  public void Stop(){
    m_intake.stopMotor();
    //m_topIntakeMotor.stopMotor();
    //m_bottomIntakeMotor.stopMotor();
  }
}