package frc.robot.subsystems.elevator;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class ElevatorConstants {

    public static final double kIsStopped = 0.002; // rotations per sec

    public static final double kStowHeight = 0;
    public static final double kStowTolerance = kStowHeight + 0.1;
    public static final double kStowVolts = -0.8;

    public static final double kPivotInterferenceLow = 0.2;
    public static final double kPivotInterferenceHigh = 1.43;

    public static final double kSupplyCurrentLimit = 60;
    public static final double kPositionTolerance = 0.02;
    public static final double kMaxElevatorHeight = 6.63;

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> new Gains(50.0, 0.0, 0.1, 0.2, 0.6, 0.0, 0.7);
                case SIMBOT -> new Gains(50, 0.0, 2.0, 0.33329, 0.00083, 0.0, 0);
                default -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0);
            };

    public static final ElevatorConfig kElevatorConfig = new ElevatorConfig(Ports.kEleLeaderID, Ports.kEleFollowerID);

    public static final double kGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> 5.0;
                case BETA -> 5.0;
                default -> 5.0;
            };

    public static final MotionMagicConfigs kMotionMagicConfigs =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> new MotionMagicConfigs(55, 30, 0);
                case BETA -> new MotionMagicConfigs(0, 0, 0);
                default -> new MotionMagicConfigs(0, 0, 0);
            };

    public record ElevatorConfig(int leaderID, int followerID) {}

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public record MotionMagicConfigs(double vel, double accel, double jerk) {}
}
