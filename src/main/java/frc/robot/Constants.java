package frc.robot;

public class Constants {
    public static final class Operators {
        public static final int DriverOne = 0;
        public static final int DriverTwo = 1;
    }
    public static final class LimeLight {
        public static final String fieldPositionFront = "limelight-front";
        public static final String fieldPositionBack = "limelight-back";
        public static final String shooterTargetingName = "limelight-shooter";
    }
    public static final class Climb {
        public static final int motorId = 15;
        public static final int canCoderId = 29;

        public static final double bottomPosition = 0;
        public static final double upPosition = 0;
        public static final double pullPosition = 0;
        public static final double topPosition = 0;
    }
    public static final class Intake {
        public static final int bedMotorId = 14;
        public static final int intakeMotorId = 1;
        public static final int hopperMotorId = 16;

        public static final double retactedLimit = 0;
        public static final double extendedLimit = 0;
    }
    public static final class Shooter {
        public static final int leftLaunchMotorId = 21;
        public static final int rightLaunchMotorId = 22;
        public static final int turretMotorId = 17;
        public static final int hoodMotorId = 19;
        public static final int feederMotorId = 18;

        public static final int hoodEncoder = 20;
        public static final int turretEncoder = 25;
        public static final int hopperEncoderId = 26;

        public static final double turretClockwiseLimit = 0;
        public static final double turretCounterClockwiseLimit = 0;

        public static final double hubBottomPosition = 0;
        public static final double hubTopPosition = 0;
    }
}