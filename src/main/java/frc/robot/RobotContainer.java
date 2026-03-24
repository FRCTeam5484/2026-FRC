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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.classes.Telemetry;
import frc.robot.classes.TunerConstants;
import frc.robot.commands.cmdAuto_AutoAlignAndShoot;
import frc.robot.commands.cmdAuto_AutoAlignShootMove;
import frc.robot.commands.cmdAuto_AutoShoot;
import frc.robot.commands.cmdAuto_ClimbLower;
import frc.robot.commands.cmdAuto_ClimbRaise;
import frc.robot.commands.cmdAuto_RelayToAlliance;
import frc.robot.commands.cmdAuto_Unjam;
import frc.robot.commands.cmdHopper_TeleOp;
import frc.robot.commands.cmdIntake_TeleOp;
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
        NamedCommands.registerCommand("Auto Align Shoot and Move", new cmdAuto_AutoAlignShootMove(drivetrain, hood, shooter, bed, feeder, intake, ()->0.2).withTimeout(4));
        NamedCommands.registerCommand("Auto Align and Shoot", new cmdAuto_AutoAlignAndShoot(drivetrain, hood, shooter, bed, feeder, intake).withTimeout(4));
        NamedCommands.registerCommand("Extend Hopper", new InstantCommand(()->hopper.ExtendHopper(), hopper));
        NamedCommands.registerCommand("Retract Hopper", new InstantCommand(()->hopper.RetractHopper(), hopper));
        
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

        /// Hopper Controls
        driverOne.a().whileTrue(new RunCommand(()->hopper.ExtendHopper(), hopper));
        driverOne.a().onFalse(new InstantCommand(()->hopper.Stop(), hopper));
        driverOne.b().whileTrue(new RunCommand(()->hopper.RetractHopper(), hopper));
        driverOne.b().onFalse(new InstantCommand(()->hopper.Stop(), hopper));
        driverOne.y().onTrue(new InstantCommand(()-> hopper.ResetEncoder(), hopper));
        driverOne.leftBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->-0.2));
        driverOne.rightBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->0.2));
        
        /// Intake Controls
        driverOne.leftTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->-.7));
        driverOne.rightTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->.7));

        /////////////////////////////////////
        /*  DriverTwo Controls for TeleOp  */
        /////////////////////////////////////
                
        // Auto Shoot
        driverTwo.a().whileTrue(new cmdAuto_AutoAlignShootMove(drivetrain, hood, shooter, bed, feeder, intake, ()->-driverOne.getLeftY()));
        driverTwo.a().onFalse(new InstantCommand(() -> hood.putHoodDown()));
        driverTwo.x().whileTrue(new cmdAuto_AutoShoot(bed, feeder, shooter, intake, 1.0));
        driverTwo.y().whileTrue(new cmdAuto_AutoShoot(bed, feeder, shooter, intake, 0.75));
        driverTwo.start().whileTrue(new cmdAuto_RelayToAlliance(hood, shooter, bed, feeder));
        driverTwo.start().onFalse(new InstantCommand(() -> hood.putHoodDown()));

        // Reset Hood Encoder
        driverTwo.back().onTrue(new InstantCommand(() -> hood.ResetEncoder()));

        // Unjam
        driverTwo.b().whileTrue(new cmdAuto_Unjam(bed, feeder, shooter));

        /// Climb Control
        climb.setDefaultCommand(Commands.run(()->climb.TeleOpNoSafe(MathUtil.applyDeadband(-driverTwo.getLeftY(), 0.05)), climb));
        driverTwo.leftBumper().whileTrue(new cmdAuto_ClimbLower(climb));
        driverTwo.rightBumper().whileTrue(new cmdAuto_ClimbRaise(climb));

        /// Hood Control
        hood.setDefaultCommand(Commands.run(()->hood.TeleOpNoSafe(MathUtil.applyDeadband(-driverTwo.getRightY(), 0.1)*0.1), hood));  
              
    }
}