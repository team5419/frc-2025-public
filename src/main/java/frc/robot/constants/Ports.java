package frc.robot.constants;

public class Ports {

    // controllers
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    // drivetrain
    public static final int kFrontLeftDriveID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 11;
                default -> 10;
            };
    public static final int kFrontLeftSteerID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 21;
                default -> 20;
            };
    public static final int kFrontLeftEncoderID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 31;
                default -> 30;
            };

    public static final int kFrontRightDriveID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 12;
                default -> 11;
            };
    public static final int kFrontRightSteerID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 22;
                default -> 21;
            };
    public static final int kFrontRightEncoderID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 32;
                default -> 31;
            };

    public static final int kBackLeftDriveID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 14;
                default -> 13;
            };
    public static final int kBackLeftSteerID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 24;
                default -> 23;
            };
    public static final int kBackLeftEncoderID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 34;
                default -> 33;
            };

    public static final int kBackRightDriveID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 13;
                default -> 12;
            };
    public static final int kBackRightSteerID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 23;
                default -> 22;
            };
    public static final int kBackRightEncoderID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 33;
                default -> 32;
            };

    public static final int kLedPort = 9;

    public static final int kPigeonID = 51;

    public static final int kRollerID =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 19;
                default -> 49;
            };

    public static final int kPivotID =
            switch (GlobalConstants.getRobotType()) {
                default -> 27;
            };

    public static final int kEleLeaderID =
            switch (GlobalConstants.getRobotType()) {
                default -> 19;
            };

    public static final int kEleFollowerID =
            switch (GlobalConstants.getRobotType()) {
                default -> 18;
            };

    public static final int kClimbLeaderID =
            switch (GlobalConstants.getRobotType()) {
                default -> 28;
            };

    public static final int kClimbFollowerID =
            switch (GlobalConstants.getRobotType()) {
                default -> 29;
            };

    public static final int kBeamBreakPort = 0;
}
