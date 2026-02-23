
package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class subIntake extends SubsystemBase {
  SparkMax intakeMotor = new SparkMax(Constants.Intake.intakeMotorId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  public subIntake() {}

  @Override
  public void periodic() {

  }

  public void TeleOp(double value){
    intakeMotor.set(value);
  }
  public void Stop(){
    intakeMotor.stopMotor();
  }
}