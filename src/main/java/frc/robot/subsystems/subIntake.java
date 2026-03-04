
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subIntake extends SubsystemBase {
  SparkMax m_topIntakeMotor = new SparkMax(Constants.Intake.topMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  SparkMax m_bottomIntakeMotor = new SparkMax(Constants.Intake.bottomMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  public subIntake() {}

  @Override
  public void periodic() {

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