package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.GlobalConstants;
import frc.robot.lib.swerve.ConstantsWrapper;
import frc.robot.subsystems.swerve.generated.TunerConstantsAlpha;
import frc.robot.subsystems.swerve.generated.TunerConstantsRadium;

public class SwerveConstants {

    public static final ConstantsWrapper TunerConstants = new ConstantsWrapper(
            switch (GlobalConstants.getRobotType()) {
                case RADIUM_PROTO -> TunerConstantsRadium.class;
                default -> TunerConstantsAlpha.class;
            });

    // TunerConstants doesn't include these constants, so they are declared locally

    public static final double kOdometryFreq =
            new CANBus(TunerConstants.getDrivetrainConstants().CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public static final double kDriveBaseRadius = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.getFrontLeft().LocationX, TunerConstants.getFrontLeft().LocationY),
                    Math.hypot(TunerConstants.getFrontRight().LocationX, TunerConstants.getFrontRight().LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.getBackLeft().LocationX, TunerConstants.getBackLeft().LocationY),
                    Math.hypot(TunerConstants.getBackRight().LocationX, TunerConstants.getBackRight().LocationY)));

    public static final double kRobotMass = 63.957;
    public static final double kRobotMoi = 3.825;
    public static final double kWheelCOF = 1.9;

    public static final double kSpeedAt12Volts =
            TunerConstants.getSpeedAt12Volts().in(MetersPerSecond);

    //    --- To configure from code ---
    public static RobotConfig kPPConfig = new RobotConfig(
            kRobotMass,
            kRobotMoi,
            new ModuleConfig(
                    TunerConstants.getFrontLeft().WheelRadius,
                    kSpeedAt12Volts,
                    kWheelCOF,
                    DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.getFrontLeft().DriveMotorGearRatio),
                    TunerConstants.getFrontLeft().SlipCurrent,
                    1),
            getModuleTranslations());

    public static final PPHolonomicDriveController kAutoController =
            new PPHolonomicDriveController(new PIDConstants(2, 0.0, 0), new PIDConstants(2, 0.0, 0.0));

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.getFrontLeft().LocationX, TunerConstants.getFrontLeft().LocationY),
            new Translation2d(TunerConstants.getFrontRight().LocationX, TunerConstants.getFrontRight().LocationY),
            new Translation2d(TunerConstants.getBackLeft().LocationX, TunerConstants.getBackLeft().LocationY),
            new Translation2d(TunerConstants.getBackRight().LocationX, TunerConstants.getBackRight().LocationY)
        };
    }
}
