package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.classes.Telemetry;
import frc.robot.classes.TunerConstants;
import frc.robot.commands.cmdAuto_AutoAlignShootMove;
import frc.robot.commands.cmdAuto_DeadcodeShoot;
import frc.robot.commands.cmdClimb_Lower;
import frc.robot.commands.cmdClimb_Raise;
import frc.robot.commands.cmdClimb_TeleOp;
import frc.robot.commands.cmdHood_Down;
import frc.robot.commands.cmdAuto_AutoRelayToAlliance;
import frc.robot.commands.cmdAuto_AutoShoot;
import frc.robot.commands.cmdAuto_Unjam;
import frc.robot.commands.cmdHood_TeleOp;
import frc.robot.commands.cmdHopper_Extend;
import frc.robot.commands.cmdHopper_Retract;
import frc.robot.commands.cmdHopper_TeleOp;
import frc.robot.commands.cmdIntake_TeleOp;
import frc.robot.commands.cmdShooter_TestRPM;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subClimb;
import frc.robot.subsystems.subDrive;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subHopper;
import frc.robot.subsystems.subIntake;
import frc.robot.subsystems.subLimelight;
import frc.robot.subsystems.subShooter;
import frc.robot.subsystems.subHood;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            // At some point, we need to change this to DriveRequestType.Velocity for closed-loop control but need to TUNE the PID
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driverOne = new CommandXboxController(0);
    private final CommandXboxController driverTwo = new CommandXboxController(1);
    private final CommandXboxController driverThree = new CommandXboxController(2);
    public final subDrive drivetrain = TunerConstants.createDrivetrain();
    public final subBed bed = new subBed();
    public final subClimb climb = new subClimb();
    public final subFeeder feeder = new subFeeder();
    public final subHopper hopper = new subHopper();
    public final subIntake intake = new subIntake();
    public final subHood hood = new subHood();
    public final subShooter shooter = new subShooter();
    public final subLimelight lime = new subLimelight(drivetrain);
    private final SendableChooser<Command> autoChooser;    

    public RobotContainer() {
        // Named Commands
        NamedCommands.registerCommand("Shooter DeadCode", new cmdAuto_DeadcodeShoot(bed, feeder, shooter, intake, 0.70).withTimeout(4));
        NamedCommands.registerCommand("Auto Shoot", new cmdAuto_AutoShoot(hood, shooter, bed, feeder).withTimeout(5));
        NamedCommands.registerCommand("Climb Raise Auto", new cmdClimb_Raise(climb).withTimeout(4));
        NamedCommands.registerCommand("Climb Lower Auto", new cmdClimb_Lower(climb).withTimeout(4));
        NamedCommands.registerCommand("Auto Align Shoot and Move", new cmdAuto_AutoAlignShootMove(drivetrain, hood, shooter, bed, feeder, intake, ()->0.2).withTimeout(5));
        NamedCommands.registerCommand("Auto Extend Hopper", new cmdHopper_Extend(hopper).withTimeout(2));
        NamedCommands.registerCommand("Auto Retract Hopper", new cmdHopper_Retract(hopper).withTimeout(2));
        NamedCommands.registerCommand("Auto Run Intake", new cmdIntake_TeleOp(intake, ()->0.7).withTimeout(5));
        
        DriverStation.silenceJoystickConnectionWarning(true);
        autoChooser = AutoBuilder.buildAutoChooser("BackShootSTOP");
        SmartDashboard.putData("Auto Mode", autoChooser);

        FollowPathCommand.warmupCommand().schedule();

        ConfigureTeleOpControls();
        //ConfigureDriverThree();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void ConfigureTeleOpControls(){
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));
        drivetrain.registerTelemetry(logger::telemeterize);

        /////////////////////////////////////
        /*  Default Controls               */
        /////////////////////////////////////

        /// Drive Default
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverOne.getLeftY()  * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverOne.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverOne.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /// Climb Default
        climb.setDefaultCommand(new cmdClimb_TeleOp(climb, ()->MathUtil.applyDeadband(-driverTwo.getLeftY(), 0.05)));

        /// Hood Default
        hood.setDefaultCommand(new cmdHood_TeleOp(hood, ()->MathUtil.applyDeadband(-driverTwo.getRightY(), 0.05))); //new cmdHood_Down(hood));


        /////////////////////////////////////
        /*  DriverOne Controls for TeleOp  */
        /////////////////////////////////////
        
        /// Reset Heading
        driverOne.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        /// Limelight Enable/Disable
        driverOne.start().onTrue(new InstantCommand(()->lime.toggleBack()));
        driverOne.back().onTrue(new InstantCommand(()->lime.toggleFront()));

        // Shooter Test
        driverOne.a().whileTrue(new cmdShooter_TestRPM(shooter));
        driverOne.a().onFalse(new InstantCommand(()->shooter.Stop(), shooter));

        /////////////////////////////////////
        /*  DriverTwo Controls for TeleOp  */
        /////////////////////////////////////
                
        // Auto Shoot
        driverTwo.a().whileTrue(new cmdAuto_AutoAlignShootMove(drivetrain, hood, shooter, bed, feeder, intake, ()->-driverOne.getLeftY()  * MaxSpeed));        
        driverTwo.x().whileTrue(new cmdAuto_DeadcodeShoot(bed, feeder, shooter, intake, 1.0));        
        driverTwo.y().whileTrue(new cmdAuto_DeadcodeShoot(bed, feeder, shooter, intake, 0.75));        
        driverTwo.start().whileTrue(new cmdAuto_AutoRelayToAlliance(hood, shooter, bed, feeder));
        
        // Reset Hood Encoder
        driverTwo.back().onTrue(new InstantCommand(() -> hood.ResetEncoder()));

        // Unjam
        driverTwo.b().whileTrue(new cmdAuto_Unjam(bed, feeder, shooter));

        /// Hopper Controls
        //driver.y().onTrue(new InstantCommand(()-> hopper.ResetEncoder(), hopper));
        driverTwo.leftBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->-0.2));
        driverTwo.leftBumper().onFalse(new InstantCommand(()->hopper.Stop(), hopper));
        driverTwo.rightBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->0.2));
        driverTwo.rightBumper().onFalse(new InstantCommand(()->hopper.Stop(), hopper));

        /// Intake Controls
        driverTwo.leftTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->.7));
        driverTwo.leftTrigger().onFalse(new InstantCommand(()->intake.Stop(), intake));
        driverTwo.rightTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->-.7));
        driverTwo.rightTrigger().onFalse(new InstantCommand(()->intake.Stop(), intake));        
        
        /// Climb Control
        //driverTwo.leftBumper().whileTrue(new cmdClimb_Lower(climb));
        //driverTwo.rightBumper().whileTrue(new cmdClimb_Raise(climb));

        /// Hood Control
        driverTwo.povUp().whileTrue(new cmdHood_TeleOp(hood, ()->0.1));
        driverTwo.povDown().whileTrue(new cmdHood_TeleOp(hood, ()->-0.1));
    }
    private void ConfigureDriverThree(){
        ////////////////////////////////////////
        /*  DriverThree Controls for Testing  */
        ////////////////////////////////////////       
        
        /// Drive Default
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverThree.getLeftY()  * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverThree.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverThree.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverThree.a().whileTrue(new RunCommand(()->shooter.TeleOp(.5), shooter));
        driverThree.a().onFalse(new InstantCommand(()->shooter.Stop(), shooter));
        driverThree.b().whileTrue(new RunCommand(()->feeder.TeleOp(.5), feeder));
        driverThree.b().onFalse(new InstantCommand(()->feeder.Stop(), feeder));
        driverThree.x().whileTrue(new RunCommand(()->hopper.TeleOp(.5), hopper));
        driverThree.x().onFalse(new InstantCommand(()->hopper.Stop(), hopper));
        driverThree.y().whileTrue(new RunCommand(()->hopper.TeleOp(-.5), hopper));
        driverThree.y().onFalse(new InstantCommand(()->hopper.Stop(), hopper));

        driverThree.povUp().whileTrue(new RunCommand(()->hood.TeleOpNoSafe(.1), hood));
        driverThree.povUp().onFalse(new InstantCommand(()->hood.Stop(), hood));
        driverThree.povDown().whileTrue(new RunCommand(()->hood.TeleOpNoSafe(-.1), hood));
        driverThree.povDown().onFalse(new InstantCommand(()->hood.Stop(), hood));

        driverThree.povLeft().whileTrue(new RunCommand(()->climb.TeleOp(.2), climb));
        driverThree.povLeft().onFalse(new InstantCommand(()->climb.Stop(), climb));
        driverThree.povRight().whileTrue(new RunCommand(()->climb.TeleOp(-.2), climb));
        driverThree.povRight().onFalse(new InstantCommand(()->climb.Stop(), climb)); 

        driverThree.leftBumper().whileTrue(new RunCommand(()->intake.TeleOp(.5), intake));
        driverThree.leftBumper().onFalse(new InstantCommand(()->intake.Stop(), intake));
        driverThree.rightBumper().whileTrue(new RunCommand(()->bed.TeleOp(.5), bed));
        driverThree.rightBumper().onFalse(new InstantCommand(()->bed.Stop(), bed));

    }
}