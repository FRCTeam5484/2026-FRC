package frc.robot;

public class Constants {
    public static final class Operators {
        public static final int DriverOne = 0;
        public static final int DriverTwo = 1;
    }
    public static final class LimeLight {
        public static final String fieldPositionFrontLeftName = "limelight-robot-frontleft";
        public static final String fieldPositionFrontRightName = "limelight-robot-frontright";
        public static final String fieldPositionRearLeftName = "limelight-robot-rearleft";
        public static final String fieldPositionRearRightName = "limelight-robot-rearright";
        public static final String shooterTargetingName = "limelight-shooter";
    }
    public static final class Climb {
        public static final int motorId = 19;

        public static final double bottomPosition = 0;
        public static final double upPosition = 0;
        public static final double pullPosition = 0;
        public static final double topPosition = 0;
    }
    public static final class Intake {
        public static final int bedMotorId = 50;
        public static final int intakeMotorId = 56;
        public static final int leftExtendMotorId = 57;
        public static final int rightExtendMotorId = 57;

        public static final double retactedLimit = 0;
        public static final double extendedLimit = 0;
    }
    public static final class Shooter {
        public static final int leftLaunchMotorId = 51;
        public static final int rightLaunchMotorId = 52;
        public static final int turretMotorId = 53;
        public static final int angleMotorId = 54;
        public static final int feederMotorId = 55;

        public static final double turretClockwiseLimit = 0;
        public static final double turretCounterClockwiseLimit = 0;

        public static final double angleBottomPosition = 0;
        public static final double angleTopPosition = 0;
    }
    public static final class Drive {
        // These are for reference only.  These IDs are actually configured in /Classes/TunerConstants.java
        public static final int frontLeftDriveMotorId = 11;
        public static final int frontLeftTurnMotorId = 12;
        public static final int frontLeftCanId = 13;

        public static final int frontRightDriveMotorId = 21;
        public static final int frontRightTurnMotorId = 22;
        public static final int frontRightCanId = 23;

        public static final int backLeftDriveMotorId = 31;
        public static final int backLeftTurnMotorId = 32;
        public static final int backLeftCanId = 33;

        public static final int backRightDriveMotorId = 41;
        public static final int backRightTurnMotorId = 42;
        public static final int backRightCanId = 43;

        public static final int PigionId = 1;
    }
}