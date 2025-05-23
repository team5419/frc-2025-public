package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import java.util.List;

public class ClimbIOTalonFX implements ClimbIO {
    private TalonFX leaderMotor, followerMotor;

    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0);

    private NeutralOut reqNeutral = new NeutralOut();

    private final List<StatusSignal<Angle>> motorPosition;
    private final List<StatusSignal<AngularVelocity>> motorVelocity;
    private final List<StatusSignal<Current>> motorCurrent;
    private final List<StatusSignal<Temperature>> motorTemperature;
    private final List<StatusSignal<Voltage>> motorVoltage;
    private final List<StatusSignal<Double>> motorReference;
    private final List<StatusSignal<Double>> motorReferenceVelocity;

    public ClimbIOTalonFX() {

        leaderMotor = new TalonFX(Ports.kClimbLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kClimbFollowerID, GlobalConstants.kCANivoreName);

        motorPosition = List.of(leaderMotor.getPosition(), followerMotor.getPosition());
        motorVoltage = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());
        motorVelocity = List.of(leaderMotor.getVelocity(), followerMotor.getVelocity());
        motorTemperature = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());
        motorCurrent = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
        motorReference = List.of(leaderMotor.getClosedLoopReference(), followerMotor.getClosedLoopReference());
        motorReferenceVelocity =
                List.of(leaderMotor.getClosedLoopReferenceSlope(), followerMotor.getClosedLoopReferenceSlope());

        followerMotor.setControl(new Follower(Ports.kClimbLeaderID, true));

        talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        talonConfig.Slot0.kP = ClimbConstants.kGains.kP();
        talonConfig.Slot0.kI = ClimbConstants.kGains.kI();
        talonConfig.Slot0.kD = ClimbConstants.kGains.kD();
        talonConfig.Slot0.kS = ClimbConstants.kGains.kS();
        talonConfig.Slot0.kV = ClimbConstants.kGains.kV();
        talonConfig.Slot0.kA = ClimbConstants.kGains.kA();
        talonConfig.Slot0.kG = ClimbConstants.kGains.kG();

        talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConfig.Feedback.SensorToMechanismRatio = ClimbConstants.kGearRatio;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity =
                Units.degreesToRotations(ClimbConstants.kMotionConfigs.kCruiseVel());
        talonConfig.MotionMagic.MotionMagicAcceleration =
                Units.degreesToRotations(ClimbConstants.kMotionConfigs.kAccel());
        talonConfig.MotionMagic.MotionMagicJerk = Units.degreesToRotations(ClimbConstants.kMotionConfigs.kJerk());
        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition.get(0),
                motorVelocity.get(0),
                motorCurrent.get(0),
                motorTemperature.get(0),
                motorVoltage.get(0),
                motorReference.get(0),
                motorReferenceVelocity.get(0),
                motorPosition.get(1),
                motorVelocity.get(1),
                motorCurrent.get(1),
                motorTemperature.get(1),
                motorVoltage.get(1),
                motorReference.get(1),
                motorReferenceVelocity.get(1),
                leaderMotor.getDutyCycle(),
                followerMotor.getDutyCycle());

        followerMotor.getConfigurator().apply(talonConfig);

        // talonConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        // talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // talonConfig.Feedback.SensorToMechanismRatio = 1.0;
        // talonConfig.Feedback.RotorToSensorRatio = ClimbConstants.kGearRatio;

        leaderMotor.getConfigurator().apply(talonConfig);

        resetPosition(ClimbConstants.kStowAngle);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.leaderMotorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(0),
                        motorVelocity.get(0),
                        motorCurrent.get(0),
                        motorTemperature.get(0),
                        motorVoltage.get(0),
                        motorReference.get(0),
                        motorReferenceVelocity.get(0))
                .isOK();

        inputs.followerMotorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(1),
                        motorVelocity.get(1),
                        motorCurrent.get(1),
                        motorTemperature.get(1),
                        motorVoltage.get(1),
                        motorReference.get(1),
                        motorReferenceVelocity.get(1))
                .isOK();

        inputs.positionDegrees = motorPosition.stream()
                .mapToDouble((s) -> Units.rotationsToDegrees(s.getValueAsDouble()))
                .toArray();

        inputs.positionReference = motorReference.stream()
                .mapToDouble((s) -> Units.rotationsToDegrees(s.getValueAsDouble()))
                .toArray();

        inputs.velocityMetersPerSecond = motorVelocity.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.motorReferenceVelocity = motorReferenceVelocity.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.appliedVolts = motorVoltage.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.supplyCurrentAmps = motorCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.tempCelsius = motorTemperature.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
    }

    @Override
    public void runPosition(double targetPositionDegrees) {
        leaderMotor.setControl(reqMotionMagic.withPosition(Units.degreesToRotations(targetPositionDegrees)));
    }

    @Override
    public void runVolts(double volts) {
        leaderMotor.setVoltage(volts);
        // followerMotor.setVoltage(volts);
    }

    @Override
    public void resetPosition(double angle) {
        leaderMotor.setPosition(Units.degreesToRotations(angle));
        followerMotor.setPosition(Units.degreesToRotations(angle));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leaderMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        talonConfig.Slot0.kP = kP;
        talonConfig.Slot0.kI = kI;
        talonConfig.Slot0.kD = kD;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }

    public void setFF(double kA, double kG, double kS, double kV) {
        talonConfig.Slot0.kA = kA;
        talonConfig.Slot0.kG = kG;
        talonConfig.Slot0.kS = kS;
        talonConfig.Slot0.kV = kV;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }

    @Override
    public void setProfile(boolean slow) {
        talonConfig.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(
                slow ? ClimbConstants.kMotionConfigsSlow.kCruiseVel() : ClimbConstants.kMotionConfigs.kCruiseVel());
        talonConfig.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(
                slow ? ClimbConstants.kMotionConfigs.kAccel() : ClimbConstants.kMotionConfigs.kAccel());
        talonConfig.MotionMagic.MotionMagicJerk = Units.degreesToRotations(
                slow ? ClimbConstants.kMotionConfigs.kJerk() : ClimbConstants.kMotionConfigs.kJerk());
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }

    @Override
    public void stop() {
        leaderMotor.setControl(reqNeutral);
    }
}
