package frc.robot.lib.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

public class ConstantsWrapper {

    // Map to hold the name/value pairs for all input class fields
    private final Map<String, Object> fieldValues;

    /**
     * Constructs a ConstantsWrapper from a given class.
     * It reads all declared fields (even private ones) using reflection.
     *
     * @param constantsClass the Class whose fields are to be captured.
     */
    public ConstantsWrapper(Class<?> constantsClass) {

        fieldValues = new HashMap<>();
        Field[] fields = constantsClass.getDeclaredFields();

        for (Field field : fields) {
            field.setAccessible(true);
            try {
                Object value = field.get(null);
                fieldValues.put(field.getName(), value);
            } catch (IllegalAccessException e) {
                e.printStackTrace(); // might want to handle this differently
                System.exit(200);
            }
        }
    }

    // Generic method for internal use
    @SuppressWarnings("unchecked")
    private <T> T getField(String fieldName, Class<T> type) {
        Object value = fieldValues.get(fieldName);
        if (value == null) {
            throw new IllegalArgumentException("Field " + fieldName + " not found.");
        }
        return (T) value;
    }

    // Gains and configuration fields
    public Slot0Configs getSteerGains() {
        return getField("steerGains", Slot0Configs.class);
    }

    public Slot0Configs getDriveGains() {
        return getField("driveGains", Slot0Configs.class);
    }

    public ClosedLoopOutputType getSteerClosedLoopOutput() {
        return getField("kSteerClosedLoopOutput", ClosedLoopOutputType.class);
    }

    public ClosedLoopOutputType getDriveClosedLoopOutput() {
        return getField("kDriveClosedLoopOutput", ClosedLoopOutputType.class);
    }

    public DriveMotorArrangement getDriveMotorType() {
        return getField("kDriveMotorType", DriveMotorArrangement.class);
    }

    public SteerMotorArrangement getSteerMotorType() {
        return getField("kSteerMotorType", SteerMotorArrangement.class);
    }

    public SteerFeedbackType getSteerFeedbackType() {
        return getField("kSteerFeedbackType", SteerFeedbackType.class);
    }

    public Current getSlipCurrent() {
        return getField("kSlipCurrent", Current.class);
    }

    public TalonFXConfiguration getDriveInitialConfigs() {
        return getField("driveInitialConfigs", TalonFXConfiguration.class);
    }

    public TalonFXConfiguration getSteerInitialConfigs() {
        return getField("steerInitialConfigs", TalonFXConfiguration.class);
    }

    public CANcoderConfiguration getEncoderInitialConfigs() {
        return getField("encoderInitialConfigs", CANcoderConfiguration.class);
    }

    public Pigeon2Configuration getPigeonConfigs() {
        return getField("pigeonConfigs", Pigeon2Configuration.class);
    }

    public CANBus getCANBus() {
        return getField("kCANBus", CANBus.class);
    }

    public LinearVelocity getSpeedAt12Volts() {
        return getField("kSpeedAt12Volts", LinearVelocity.class);
    }

    public double getCoupleRatio() {
        return getField("kCoupleRatio", Double.class);
    }

    public double getDriveGearRatio() {
        return getField("kDriveGearRatio", Double.class);
    }

    public double getSteerGearRatio() {
        return getField("kSteerGearRatio", Double.class);
    }

    public Distance getWheelRadius() {
        return getField("kWheelRadius", Distance.class);
    }

    public boolean getInvertLeftSide() {
        return getField("kInvertLeftSide", Boolean.class);
    }

    public boolean getInvertRightSide() {
        return getField("kInvertRightSide", Boolean.class);
    }

    public int getPigeonId() {
        return getField("kPigeonId", Integer.class);
    }

    public MomentOfInertia getSteerInertia() {
        return getField("kSteerInertia", MomentOfInertia.class);
    }

    public MomentOfInertia getDriveInertia() {
        return getField("kDriveInertia", MomentOfInertia.class);
    }

    public Voltage getSteerFrictionVoltage() {
        return getField("kSteerFrictionVoltage", Voltage.class);
    }

    public Voltage getDriveFrictionVoltage() {
        return getField("kDriveFrictionVoltage", Voltage.class);
    }

    // Swerve drivetrain and module constants fields
    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return getField("DrivetrainConstants", SwerveDrivetrainConstants.class);
    }

    @SuppressWarnings("unchecked")
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontLeft() {
        return getField("FrontLeft", SwerveModuleConstants.class);
    }

    @SuppressWarnings("unchecked")
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontRight() {
        return getField("FrontRight", SwerveModuleConstants.class);
    }

    @SuppressWarnings("unchecked")
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackLeft() {
        return getField("BackLeft", SwerveModuleConstants.class);
    }

    @SuppressWarnings("unchecked")
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackRight() {
        return getField("BackRight", SwerveModuleConstants.class);
    }

    /**
     * Returns all field name/value pairs.
     *
     * @return a Map containing all captured fields.
     */
    public Map<String, Object> getAllFields() {
        return new HashMap<>(fieldValues);
    }
}
