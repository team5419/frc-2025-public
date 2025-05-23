package frc.robot.subsystems.roller;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class RollerConstants {
    public static final RollerConfig kRollerConfig = new RollerConfig(Ports.kRollerID);

    public static final double kRollerGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> 1.0;
                default -> 1.0;
            };

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> new Gains(0.3, 0.0, 0.0, 0.33329, 0.12, 0.0);
                case ALPHA -> new Gains(0.5, 0.0, 0.0, 0.26, 0.125, 0.0);
                case SIMBOT -> new Gains(0.010, 0.008, 0.00015, 0.01, 0.1, 0.1);
                default -> new Gains(0.2, 0, 0, 0.2, 0, 0);
            };

    public static final double kRollerFrequency = 50.0;
    public static final double kVelocityTolerance = 1.0;
    public static final double kSupplyCurrentLimit = 40.0;
    public static final double kSimBreamBreakDelay = 0.2;

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public record RollerConfig(int ID) {}
}
