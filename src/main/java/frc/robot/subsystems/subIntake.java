
package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subIntake extends SubsystemBase {
  SparkMax m_topIntakeMotor = new SparkMax(Constants.Intake.topMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  SparkMax m_bottomIntakeMotor = new SparkMax(Constants.Intake.bottomMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

  public subIntake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kCoast)
      .inverted(false);
    
      m_topIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_bottomIntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Intake Top Motor Output", m_topIntakeMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Intake Bottom Motor Output", m_bottomIntakeMotor.getAppliedOutput());
  }

  public void TeleOp(double value){
    m_topIntakeMotor.set(value);
    m_bottomIntakeMotor.set(value);
  }
  public void Stop(){
    m_topIntakeMotor.stopMotor();
    m_bottomIntakeMotor.stopMotor();
  }
}