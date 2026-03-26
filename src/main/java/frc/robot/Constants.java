package frc.robot;

public class Constants {
    public static final class TargetingDistance{
        public static final double minDistance = -11;
        public static final double maxDistance = 12;
    }
    public static final class Bed{
        public static final int motorId = 14;
    }
    public static final class Climb {
        public static final int motorId = 15;

        public static final int topLimitSwitchId = 0;
        public static final int bottomLimitSwitchId = 1;
    }
    public static final class Feeder {
        public static final int motorId = 18;
    }
    public static final class Hood{
        public static final int motorId = 19;
        public static final int canEncoderId = 20;

        public static final double topPosition = 0.5;
        public static final double bottomPosition = 0.1;
        public static final double closed = -0.08;
    }
    public static final class Hopper{
        public static final int motorId = 16;
        public static final double retactedLimit = -1.5;
        public static final double extendedLimit = -8.5;
    }
    public static final class Intake {
        public static final int x44MotorId = 23;
        public static final int topMotorId = 1;
        public static final int bottomMotorId = 2;
    }
    public static final class LimeLight {
        public static final String fieldPositionFrontLeft = "limelight-front";
        public static final String fieldPositionBackRight = "limelight-back";
        public static final String shooterTargetingName = "limelight-shooter";
    }
    public static final class Shooter {
        public static final int leftMotorId = 21;
        public static final int rightMotorId = 22;
    }
}