package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.GlobalConstants;

public class PivotConstants {

    public static final double kZeroDegrees = 1.5;
    public static final double kTopDegrees = kZeroDegrees + 65;
    public static final double kBottomDegrees = kZeroDegrees - 16;
    public static double kPivotSafeToMoveAngle = kBottomDegrees + 20;

    public static final double kAngleTolerance = 1.5;

    public static final Translation3d kPivotOrigin = new Translation3d(-0.24, 0.0, 0.5);

    public static final double kGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> 14;
                default -> 1.0;
            };

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> new Gains(175, 0, 0, 0.15, 2.5, 0, 0.08);
                case SIMBOT -> new Gains(10, 0.0, 0.0, 8.4, 0.0, 0.0, 22.9);
                default -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0);
            };

    public static MotionConfigs kMotionConfigs =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> new MotionConfigs(8000, 7200, 0); // 8000 7200
                default -> new MotionConfigs(2000 * 0.80, 800, (2000 * 0.80) * 3);
            };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}
}
