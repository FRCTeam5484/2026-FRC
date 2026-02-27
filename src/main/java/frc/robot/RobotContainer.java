package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.classes.LimelightHelpers;
import frc.robot.classes.Telemetry;
import frc.robot.classes.TunerConstants;
import frc.robot.commands.cmdAuto_AutoShoot;
import frc.robot.commands.cmdBed_TeleOp;
import frc.robot.commands.cmdClimb_TeleOp;
import frc.robot.commands.cmdFeeder_TeleOp;
import frc.robot.commands.cmdHood_TeleOp;
import frc.robot.commands.cmdHopper_TeleOp;
import frc.robot.commands.cmdIntake_TeleOp;
import frc.robot.commands.cmdTurret_TeleOp;
import frc.robot.commands.cmdShooter_TeleOp;
import frc.robot.subsystems.subBed;
import frc.robot.subsystems.subClimb;
import frc.robot.subsystems.subDrive;
import frc.robot.subsystems.subFeeder;
import frc.robot.subsystems.subHopper;
import frc.robot.subsystems.subIntake;
import frc.robot.subsystems.subLimeLight;
import frc.robot.subsystems.subShooter;
import frc.robot.subsystems.subTurret;
import frc.robot.subsystems.subHood;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driverOne = new CommandXboxController(0);
    private final CommandXboxController driverTwo = new CommandXboxController(1);
    public final subLimeLight limelight = new subLimeLight();
    public final subDrive drivetrain = TunerConstants.createDrivetrain();
    public final subBed bed = new subBed();
    public final subClimb climb = new subClimb();
    public final subFeeder feeder = new subFeeder();
    public final subHopper hopper = new subHopper();
    public final subIntake intake = new subIntake();
    public final subHood hood = new subHood();
    public final subTurret turret = new subTurret();
    public final subShooter shooter = new subShooter();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        limelight.configureFrontLeft();
        limelight.configureBackRight();
        
        DriverStation.silenceJoystickConnectionWarning(true);
        autoChooser = AutoBuilder.buildAutoChooser("Cross Line");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureDriverOneControls();

        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureDriverOneControls() {
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void ConfigureTeleOpControls(){
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));
        drivetrain.registerTelemetry(logger::telemeterize);

        // DriverOne Controls
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverOne.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverOne.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverOne.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // DriverTwo Controls

    }
    public void ConfigureTestControls(){
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));
        drivetrain.registerTelemetry(logger::telemeterize);

        /////////////////////////
        /*  DriverOne Controls */
        /////////////////////////
        /// 
        /// Drive Controls
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverOne.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverOne.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverOne.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        /// Intake Controls
        //driverOne.leftTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->driverOne.getLeftTriggerAxis()));
        driverOne.rightTrigger().whileTrue(new cmdIntake_TeleOp(intake, ()->-.25));

        /// Hopper Controls
        driverOne.leftBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->-0.2));
        driverOne.rightBumper().whileTrue(new cmdHopper_TeleOp(hopper, ()->0.2));

        /////////////////////////
        /*  DriverTwo Controls */
        /////////////////////////
        
        /// Bed Controls
        driverTwo.a().whileTrue(new cmdBed_TeleOp(bed, ()->1));
        driverTwo.b().whileTrue(new cmdBed_TeleOp(bed, ()->-1));
        driverTwo.b().whileTrue(new cmdFeeder_TeleOp(feeder, ()->-1));
        
        //driverTwo.a().whileTrue(new cmdHopper_TeleOp(hopper, ()->-0.2));
        //driverTwo.b().whileTrue(new cmdHopper_TeleOp(hopper, ()->0.2));

        /// Climb Control
        climb.setDefaultCommand(new cmdClimb_TeleOp(climb, ()->-driverTwo.getLeftY()));

        /// Turret Control
        driverTwo.leftBumper().whileTrue(new cmdTurret_TeleOp(turret, ()->-1));
        driverTwo.rightBumper().whileTrue(new cmdTurret_TeleOp(turret, ()->1));

        /// Hood Control
        hood.setDefaultCommand(new cmdHood_TeleOp(hood, ()->-driverTwo.getRightY()));

        /// Shooter Control
        driverTwo.rightTrigger().whileTrue(new cmdShooter_TeleOp(shooter, ()->driverTwo.getRightTriggerAxis()));

        /// Feeder Control 
        driverTwo.leftTrigger().whileTrue(new cmdFeeder_TeleOp(feeder, ()->1));

        /// Auto Functions
        driverTwo.x().whileTrue(new cmdAuto_AutoShoot(bed, feeder, shooter));
        driverTwo.y().whileTrue(new cmdIntake_TeleOp(intake, ()->-.22));
    }
}