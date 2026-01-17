package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Operators;
import frc.robot.subsystems.subClimb;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subIntake;
import frc.robot.subsystems.subShooter;

public class RobotContainer {
  private SendableChooser<Command> chooser = new SendableChooser<>();
  public final subClimb climb = new subClimb();
  public final subFeeder feeder = new subFeeder();
  public final subIntake intake = new subIntake();
  public final subShooter shooter = new subShooter();
  public final PowerDistribution pdh = new PowerDistribution();
  private final CommandXboxController driverOne = new CommandXboxController(Operators.DriverOne);
  private final CommandXboxController driverTwo = new CommandXboxController(Operators.DriverTwo);
  
  public RobotContainer() {
    DriverOneControls();
    DriverTwoControls();
    addAutoOptions();     
  }

  private void DriverOneControls(){
    
  }

  private void DriverTwoControls(){
    
  }
  private void addAutoOptions(){
    
  }
  public Command getAutonomousCommand() {
    //return new cmdAutonomous_Crossline(swerve); 
    return chooser.getSelected();
  }
}