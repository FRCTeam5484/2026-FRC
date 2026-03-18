package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.classes.LimelightHelpers;
import frc.robot.classes.Telemetry;
import frc.robot.classes.TunerConstants;
import frc.robot.commands.cmdAuto_AutoAlignAndShoot;
import frc.robot.commands.cmdAuto_AutoShoot;
import frc.robot.commands.cmdAuto_ClimbLower;
import frc.robot.commands.cmdAuto_ClimbRaise;
import frc.robot.commands.cmdAuto_HopperExtend;
import frc.robot.commands.cmdAuto_RelayToAlliance;
import frc.robot.commands.cmdAuto_Unjam;
import frc.robot.commands.cmdBed_TeleOp;
import frc.robot.commands.cmdClimb_TeleOp;
import frc.robot.commands.cmdFeeder_TeleOp;
import frc.robot.commands.cmdHood_TeleOp;
import frc.robot.commands.cmdHopper_TeleOp;
import frc.robot.commands.cmdIntake_TeleOp;
import frc.robot.commands.cmdShooter_TeleOp;
import frc.robot.commands.cmdTest_DriveBack;
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
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driverOne = new CommandXboxController(0);
    private final CommandXboxController driverTwo = new CommandXboxController(1);
    private final CommandXboxController driverThree = new CommandXboxController(2);
    //public final subLimeLight limelight = new subLimeLight();
    public final subDrive drivetrain = TunerConstants.createDrivetrain();
    public final subBed bed = new subBed();
    public final subClimb climb = new subClimb();
    public final subFeeder feeder = new subFeeder();
    public final subHopper hopper = new subHopper();
    public final subIntake intake = new subIntake();
    public final subHood hood = new subHood();
    public final subShooter shooter = new subShooter();
    public final subLimelight frontLimeLight = new subLimelight(Constants.LimeLight.fieldPositionFrontLeft, drivetrain);
    public final subLimelight backLimeLight = new subLimelight(Constants.LimeLight.fieldPositionBackRight, drivetrain);
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        // Named Commands
        NamedCommands.registerCommand("Shooter Auto", new cmdAuto_AutoShoot(bed, feeder, shooter, intake, 0.70).withTimeout(4));
        NamedCommands.registerCommand("Climb Raise Auto", new cmdAuto_ClimbRaise(climb).withTimeout(2));
        NamedCommands.registerCommand("Climb Lower Auto", new cmdAuto_ClimbLower(climb).withTimeout(3));
        
        DriverStation.silenceJoystickConnectionWarning(true);
        autoChooser = AutoBuilder.buildAutoChooser("ShootNoMove");
        SmartDashboard.putData("Auto Mode", autoChooser);

        FollowPathCommand.warmupCommand().schedule();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void ConfigureTeleOpControls(){
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));
        drivetrain.registerTelemetry(logger::telemeterize);

        /////////////////////////////////////
        /*  DriverOne Controls for TeleOp  */
        /////////////////////////////////////
        /// 
        /// Drive Controls
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverOne.getLeftY()  * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverOne.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverOne.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        /// Reset Heading
        driverOne.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverOne.y().whileTrue(new cmdTest_DriveBack(drivetrain));

        /// Hopper Controls
        driverOne.b().whileTrue(new cmdAuto_HopperExtend(hopper, -1.5));
        driverOne.a().whileTrue(new cmdAuto_HopperExtend(hopper, -8.5));
        driverOne.leftBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->-0.2));
        driverOne.rightBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->0.2));

        /// Intake Controls
        driverOne.leftTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->-.7));
        driverOne.rightTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->.7));

        /////////////////////////////////////
        /*  DriverTwo Controls for TeleOp  */
        /////////////////////////////////////
        /// 
        
        // Auto Shoot
        driverTwo.a().whileTrue(new cmdAuto_AutoAlignAndShoot(drivetrain, hood, shooter, bed, feeder, intake));
        driverTwo.a().onFalse(new InstantCommand(() -> hood.setPosition(-0.08)));
        driverTwo.x().whileTrue(new cmdAuto_AutoShoot(bed, feeder, shooter, intake, 1.0));
        driverTwo.y().whileTrue(new cmdAuto_AutoShoot(bed, feeder, shooter, intake, 0.75));
        driverTwo.start().whileTrue(new cmdAuto_RelayToAlliance(hood, shooter, bed, feeder));
        driverTwo.start().onFalse(new InstantCommand(() -> hood.setPosition(-0.08)));

        // Reset Hood Encoder
        driverTwo.back().onTrue(new InstantCommand(() -> hood.ResetEncoder()));

        // Unjam
        driverTwo.b().whileTrue(new cmdAuto_Unjam(bed, feeder, shooter));

        /// Climb Control
        climb.setDefaultCommand(new cmdClimb_TeleOp(climb, ()-> MathUtil.applyDeadband(-driverTwo.getLeftY(), 0.05)));
        driverTwo.leftBumper().whileTrue(new cmdAuto_ClimbLower(climb));
        driverTwo.rightBumper().whileTrue(new cmdAuto_ClimbRaise(climb));

        /// Hood Control
        hood.setDefaultCommand(Commands.run(()->hood.TeleOpNoSafe(MathUtil.applyDeadband(-driverTwo.getRightY(), 0.1)*0.1), hood));  
              
    }
    public void ConfigureTestControls(){
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

        /// Drive Test Controls
        /// 
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverThree.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverThree.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverThree.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        /// Intake Test Controls
        driverThree.leftTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->.2));
        driverThree.rightTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->-.2));

        /// Hopper Test Controls
        driverThree.leftBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->-0.2));
        driverThree.rightBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->0.2));

        /// Bed Test Controls
        //driverThree.a().whileTrue(new cmdBed_TeleOp(bed, ()->0.2));
        //driverThree.b().whileTrue(new cmdBed_TeleOp(bed, ()->-0.2));
        
        /// Feeder Test Controls
        driverThree.povUp().whileTrue(new cmdFeeder_TeleOp(feeder, ()->0.2));
        driverThree.povDown().whileTrue(new cmdFeeder_TeleOp(feeder, ()->-0.2));

        /// Shooter Test Controls
        driverThree.povLeft().whileTrue(new cmdShooter_TeleOp(shooter, ()->0.2));
        driverThree.povRight().whileTrue(new cmdShooter_TeleOp(shooter, ()->-0.2));
        
        /// Climb Test Controls
        driverThree.x().whileTrue(new cmdClimb_TeleOp(climb, ()->-0.2));
        driverThree.y().whileTrue(new cmdClimb_TeleOp(climb, ()->0.2));
        
        /// Hood Test Control
        driverThree.back().whileTrue(new InstantCommand(() -> hood.TeleOpNoSafe(-0.1)));
        driverThree.back().whileFalse(new InstantCommand(() -> hood.Stop()));
        driverThree.start().whileTrue(new InstantCommand(() -> hood.TeleOpNoSafe(0.1)));
        driverThree.start().whileFalse(new InstantCommand(() -> hood.Stop()));

        //driverThree.a().whileTrue(new cmdAuto_AutoAlignAndShoot(drivetrain, hood, shooter));
        //driverThree.a().onFalse(new InstantCommand(() -> hood.setPosition(-0.08)));
        //driverThree.b().onTrue(new InstantCommand(() -> hood.ResetEncoder()));
    }
}