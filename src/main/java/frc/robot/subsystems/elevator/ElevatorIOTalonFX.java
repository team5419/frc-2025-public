package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import java.util.List;
import lombok.Getter;

public class ElevatorIOTalonFX implements ElevatorIO {
    @Getter
    private TalonFX leaderMotor, followerMotor;
    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    private final List<StatusSignal<Angle>> motorPosition;
    private final List<StatusSignal<AngularVelocity>> motorVelocity;
    private final List<StatusSignal<Double>> motorReferencePosition;
    private final List<StatusSignal<Double>> motorReferenceVelocity;
    private final List<StatusSignal<Double>> motorReferenceError;
    private final List<StatusSignal<Voltage>> motorAppliedVoltage;
    private final List<StatusSignal<Current>> motorSupplyCurrent;
    private final List<StatusSignal<Current>> motorTorqueCurrent;
    private final List<StatusSignal<Temperature>> motorTempCelsius;

    private MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0).withUpdateFreqHz(0);
    private VoltageOut reqVoltage = new VoltageOut(0);
    private VelocityVoltage reqVelocity = new VelocityVoltage(0);
    private NeutralOut reqNeutral = new NeutralOut();
    private Follower follow = new Follower(kElevatorConfig.leaderID(), true);

    public ElevatorIOTalonFX() {

        leaderMotor = new TalonFX(kElevatorConfig.leaderID(), GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(kElevatorConfig.followerID(), GlobalConstants.kCANivoreName);

        talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        talonConfig.Slot0.kP = kGains.kP();
        talonConfig.Slot0.kI = kGains.kI();
        talonConfig.Slot0.kD = kGains.kD();
        talonConfig.Slot0.kS = kGains.kS();
        talonConfig.Slot0.kG = kGains.kG();
        talonConfig.Slot0.kV = kGains.kV();
        talonConfig.Slot0.kA = kGains.kA();
        talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        talonConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        talonConfig.Feedback.SensorToMechanismRatio = kGearRatio;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = kMotionMagicConfigs.vel();
        talonConfig.MotionMagic.MotionMagicAcceleration = kMotionMagicConfigs.accel();
        talonConfig.MotionMagic.MotionMagicJerk = kMotionMagicConfigs.jerk();

        talonConfig.CurrentLimits.StatorCurrentLimit = kSupplyCurrentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);

        motorPosition = List.of(leaderMotor.getPosition(), followerMotor.getPosition());
        motorVelocity = List.of(leaderMotor.getVelocity(), followerMotor.getVelocity());
        motorReferencePosition = List.of(leaderMotor.getClosedLoopReference(), followerMotor.getClosedLoopReference());
        motorReferenceVelocity =
                List.of(leaderMotor.getClosedLoopReferenceSlope(), followerMotor.getClosedLoopReferenceSlope());
        motorReferenceError = List.of(leaderMotor.getClosedLoopError(), followerMotor.getClosedLoopError());
        motorAppliedVoltage = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());
        motorSupplyCurrent = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
        motorTorqueCurrent = List.of(leaderMotor.getTorqueCurrent(), followerMotor.getTorqueCurrent());
        motorTempCelsius = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition.get(0),
                motorPosition.get(1),
                motorVelocity.get(0),
                motorVelocity.get(1),
                motorReferencePosition.get(0),
                motorReferencePosition.get(1),
                motorReferenceVelocity.get(0),
                motorReferenceVelocity.get(1),
                motorReferenceError.get(0),
                motorReferenceError.get(1),
                motorAppliedVoltage.get(0),
                motorAppliedVoltage.get(1),
                motorSupplyCurrent.get(0),
                motorSupplyCurrent.get(1),
                motorTorqueCurrent.get(0),
                motorTorqueCurrent.get(1),
                motorTempCelsius.get(0),
                motorTempCelsius.get(1),
                leaderMotor.getDutyCycle());

        resetPosition(0);
        followerMotor.setControl(follow);
        leaderMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leaderMotorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(0),
                        motorVelocity.get(0),
                        motorReferencePosition.get(0),
                        motorReferenceVelocity.get(0),
                        motorReferenceError.get(0),
                        motorAppliedVoltage.get(0),
                        motorSupplyCurrent.get(0),
                        motorTorqueCurrent.get(0),
                        motorTempCelsius.get(0))
                .isOK();

        inputs.followerMotorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(1),
                        motorVelocity.get(1),
                        motorReferencePosition.get(1),
                        motorReferenceVelocity.get(1),
                        motorReferenceError.get(1),
                        motorAppliedVoltage.get(1),
                        motorSupplyCurrent.get(1),
                        motorTorqueCurrent.get(1),
                        motorTempCelsius.get(1))
                .isOK();

        inputs.position = motorPosition.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.velocityRotationsPerSecond = motorVelocity.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.referencePosition = motorReferencePosition.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.referenceVelocity = motorReferenceVelocity.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.referenceError = motorReferenceError.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.appliedVolts = motorAppliedVoltage.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.supplyCurrentAmps = motorSupplyCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.statorCurrentAmps = motorTorqueCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.tempCelsius = motorTempCelsius.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
    }

    @Override
    public void resetPosition(double pos) {
        leaderMotor.setPosition(pos);
        followerMotor.setPosition(pos);
    }

    @Override
    public void runPosition(double elevatorGoal, double feedforward) {
        leaderMotor.setControl(reqMotionMagic.withPosition(elevatorGoal));
        // apply(reqMotionMagic.withPosition(elevatorGoal).withFeedForward(feedforward));
    }

    @Override
    public void runVolts(double volts) {
        leaderMotor.setControl(reqVoltage.withOutput(volts));
    }

    @Override
    public void runVelocity(double velocity) {
        // leaderMotor.setControl(reqVelocity.withVelocity(velocity));
        apply(reqVelocity.withVelocity(velocity));
    }

    @Override
    public void stop() {
        // leaderMotor.setControl(reqNeutral);
        apply(reqNeutral);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leaderMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        // followerMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private void apply(ControlRequest req) {
        leaderMotor.setControl(req);
        // followerMotor.setControl(req);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        talonConfig.Slot0.kP = kP;
        talonConfig.Slot0.kI = kI;
        talonConfig.Slot0.kD = kD;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {
        talonConfig.Slot0.kS = kS;
        talonConfig.Slot0.kG = kG;
        talonConfig.Slot0.kV = kV;
        talonConfig.Slot0.kA = kA;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }
}
