package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GlobalConstants;

public class AprilTagVisionConstants {
    public static final boolean kUsingVision = true;

    public static final double kCameraQuantity =
            switch (GlobalConstants.getRobotType()) {
                case ALPHA -> 2;
                default -> 2;
            };

    public static final String[] kCameraNames = {"OV9281_LEFT", "OV9281_RIGHT", "OV9782_3"};

    public static final String BACK_CAMERA_NAME = "OV9782_3";

    // RADIUM ROTATIONS/TRANSFORMS
    public static final Rotation3d[] kCameraRotations =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> new Rotation3d[] {
                    new Rotation3d(0, 0, Units.degreesToRadians(207)), new Rotation3d(0, 0, 0)
                };

                case ALPHA -> new Rotation3d[] {
                    new Rotation3d(0, 0, Units.degreesToRadians(-20)),
                    new Rotation3d(0, 0, Units.degreesToRadians(20)),
                    new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(180))
                };

                default -> new Rotation3d[] {new Rotation3d(), new Rotation3d()};
            };

    public static final Transform3d[] kCameraTransform =
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> new Transform3d[] {
                    new Transform3d(
                            Units.inchesToMeters(-12.028),
                            Units.inchesToMeters(3.444),
                            Units.inchesToMeters(-18.338),
                            kCameraRotations[0]),
                    new Transform3d( // Swerve
                            Units.inchesToMeters(11.335),
                            Units.inchesToMeters(11.10),
                            Units.inchesToMeters(5.045),
                            kCameraRotations[1])
                };

                case ALPHA -> new Transform3d[] {
                    new Transform3d(
                            Units.inchesToMeters(7),
                            Units.inchesToMeters(12 + 0.165 + 0.1654),
                            Units.inchesToMeters(11.5 - 0.472 - 0.472),
                            kCameraRotations[0]),
                    new Transform3d(
                            Units.inchesToMeters(7),
                            Units.inchesToMeters(-12 - 0.165 - 0.1654),
                            Units.inchesToMeters(11.5 - 0.472 - 0.472),
                            kCameraRotations[1]),
                    new Transform3d(
                            Units.inchesToMeters(-11.216),
                            Units.inchesToMeters(-10.278),
                            Units.inchesToMeters(8),
                            kCameraRotations[2])
                };

                default -> new Transform3d[] {new Transform3d(), new Transform3d()};
            };

    public static final double kAmbiguityRejectionLimit = 0.25;
    public static final double kDistanceRejectionLimit = 3;
    public static final double kAreaMinimum = 0; // 0-100%

    public static final double kMaxVelocity = 5.4; // m / s
    public static final double kMaxRotationalVelocity = Math.PI; // rad / s - 45 degrees/sec

    public static final double kTranslationalStdDevMultiplier = 0.5;
    public static final double kRotationalStdDevMultiplier = 0.5; // Was 0.00125

    // used when there is a large delta in time between estimates
    public static final double kHighTrustTranslationalStdDevMultiplier = 0.08;
    public static final double kHighTrustRotationalStdDevMultiplier = 0.08;

    /** If the delta between vision readings is higher than this value, we start weighting readings higher after we get vision back */
    public static final double kMissingReadingsDelta = 1; // seconds
    /** The amount of time that we weight vision readings higher for after a large delta */
    public static final double kStopHighTrust = 0.2; // seconds

    public static final double kMinimumVisionStdDev = 0.02;
    public static final double kAverageDistanceStdDevMultiplier = 0.02;
}
