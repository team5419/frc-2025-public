package frc.robot.subsystems.climb;

import frc.robot.constants.GlobalConstants;

public class ClimbConstants {

    public static final double kAngleTolerance = 0.7; // previously 1.5
    /** A difference greater than this means the difference between motor velocities is too big */
    public static final double kDeltaThreshold = 3;

    public static final double kStowAngle = -25;

    public static final double kGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> 400;
                case BETA -> 240;
                default -> 210;
            };

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> new Gains(100, 0, 0, 0.1, 24, 0, 0);
                case BETA -> new Gains(0, 0, 0, 0, 0, 0, 0); // TODO: tune
                case SIMBOT -> new Gains(0.010, 0.008, 0.00015, 0, 0, 0, 0);
                default -> new Gains(0.2, 0, 0, 0, 0, 0, 0);
            };

    public static final MotionConfigs kMotionConfigs =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> new MotionConfigs(150, 400, 0);
                case BETA -> new MotionConfigs(0, 0, 0);
                case SIMBOT -> new MotionConfigs(800, 2000 * 0.80, 3);
                default -> new MotionConfigs(800, 2000 * 0.80, 3);
            };

    public static final MotionConfigs kMotionConfigsSlow =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> new MotionConfigs(50, 200, 0);
                case BETA -> new MotionConfigs(0, 0, 0);
                case SIMBOT -> new MotionConfigs(800, 2000 * 0.80, 3);
                default -> new MotionConfigs(800, 2000 * 0.80, 3);
            };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public record MotionConfigs(double kCruiseVel, double kAccel, double kJerk) {}
}
