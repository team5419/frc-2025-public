package frc.robot.constants;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A class to hold the constants for the entire robot, as well as the mode of the robot (real, sim, replay)
 */
public class GlobalConstants {

    private static RobotType kRobotType = RobotType.ALPHA;
    private static Alert invalidRobotAlert =
            new Alert("Invalid robot selected, using alpha bot as default.", AlertType.kError);

    public static final double kLooperDT = 0.02;
    public static final double kLooperHZ = 50;
    /** This constant is used in LoggedTunableNumber.java */
    public static final boolean kTuningMode = true;
    /** Set this to true to use dev bindings instead of normal bindings */
    public static final boolean kDevMode = false;

    public static final double kCANErrorTimeThreshold = 0.5;
    public static final double kCANivoreErrorTimeThreshold = 0.5;
    public static final double kLowBatteryVoltage = 11.8;
    public static final double kLowBatteryDisabledTime = 1.5;
    public static final double kBrownoutVoltage = 6.0;
    public static final double kOverrideJoystick = 0.7;

    public static final String kRIOName = "rio";
    public static final String kCANivoreName =
            switch (kRobotType) {
                case RADIUM_PROTO -> "DrivebaseCanivore";
                case ALPHA -> "DrivebaseCanivore";
                default -> "Drivebase";
            };

    public static RobotType getRobotType() {
        if (RobotBase.isReal() && kRobotType == RobotType.SIMBOT) {
            invalidRobotAlert.set(true);
            kRobotType = RobotType.ALPHA;
        } else if (RobotBase.isSimulation() && kRobotType != RobotType.SIMBOT) {
            kRobotType = RobotType.SIMBOT;
        }
        return kRobotType;
    }

    public static Mode getMode() {
        return switch (kRobotType) {
            case SIMBOT -> Mode.SIM;
            default -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
        };
    }

    public enum RobotType {
        SIMBOT,
        ORBIT,
        RADIUM_PROTO,
        ALPHA,
        BETA
    }

    public enum Mode {
        REAL,
        REPLAY,
        SIM
    }

    public static void main(String... args) {
        if (kRobotType == RobotType.SIMBOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + kRobotType);
            System.exit(1);
        }
    }
}
