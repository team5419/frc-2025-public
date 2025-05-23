package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class PivotIOTalonFX implements PivotIO {
    private TalonFX pivotMotor;
    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0);

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Temperature> motorTempCelsius;
    private final StatusSignal<Current> motorSupplyCurrentAmps;
    private final StatusSignal<Double> motorReference;
    private final StatusSignal<Double> motorReferenceVelocity;

    private final VoltageOut voltageControl =
            new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

    public PivotIOTalonFX() {

        pivotMotor = new TalonFX(Ports.kPivotID);

        motorPosition = pivotMotor.getPosition();
        motorAppliedVoltage = pivotMotor.getMotorVoltage();
        motorVelocity = pivotMotor.getVelocity();
        motorTempCelsius = pivotMotor.getDeviceTemp();
        motorSupplyCurrentAmps = pivotMotor.getSupplyCurrent();
        motorReference = pivotMotor.getClosedLoopReference();
        motorReferenceVelocity = pivotMotor.getClosedLoopReferenceSlope();

        talonConfig.Slot0.kP = PivotConstants.kGains.kP();
        talonConfig.Slot0.kI = PivotConstants.kGains.kI();
        talonConfig.Slot0.kD = PivotConstants.kGains.kD();
        talonConfig.Slot0.kA = PivotConstants.kGains.kA();
        talonConfig.Slot0.kG = PivotConstants.kGains.kG();
        talonConfig.Slot0.kS = PivotConstants.kGains.kS();
        talonConfig.Slot0.kV = PivotConstants.kGains.kV();

        talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConfig.Feedback.SensorToMechanismRatio = kGearRatio;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity =
                Units.degreesToRotations(PivotConstants.kMotionConfigs.kCruiseVel());
        talonConfig.MotionMagic.MotionMagicAcceleration =
                Units.degreesToRotations(PivotConstants.kMotionConfigs.kAcceleration());
        talonConfig.MotionMagic.MotionMagicJerk = Units.degreesToRotations(PivotConstants.kMotionConfigs.kJerk());

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorTempCelsius,
                motorReference,
                motorReferenceVelocity);

        pivotMotor.getConfigurator().apply(talonConfig);

        resetPosition(kTopDegrees);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        motorPosition,
                        motorVelocity,
                        motorAppliedVoltage,
                        motorTempCelsius,
                        motorSupplyCurrentAmps,
                        motorReference,
                        motorReferenceVelocity)
                .isOK();
        inputs.position = Units.rotationsToDegrees(motorPosition.getValueAsDouble());
        inputs.appliedVolts = motorAppliedVoltage.getValueAsDouble();
        inputs.velocity = Units.rotationsToDegrees(motorVelocity.getValueAsDouble());
        inputs.tempCelcius = motorTempCelsius.getValueAsDouble();
        inputs.supplyCurrentAmps = motorSupplyCurrentAmps.getValueAsDouble();
        inputs.supplyCurrentAmps = motorSupplyCurrentAmps.getValueAsDouble();
        inputs.supplyCurrentAmps = motorSupplyCurrentAmps.getValueAsDouble();
        inputs.referencePose = Units.rotationsToDegrees(motorReference.getValueAsDouble());
        inputs.referenceVelocity = Units.rotationsToDegrees(motorReferenceVelocity.getValueAsDouble());

        // Logger.recordOutput(
        //         "DEBUG/Velocity Ref",
        //         Units.rotationsToDegrees(
        //                 pivotMotor.getClosedLoopReferenceSlope().getValueAsDouble()));
        // Logger.recordOutput(
        //         "DEBUG/Target Ref",
        //         Units.rotationsToDegrees(pivotMotor.getClosedLoopReference().getValueAsDouble()));
    }

    @Override
    public void runPosition(double goal) {
        pivotMotor.setControl(reqMotionMagic.withPosition(Units.degreesToRotations(goal)));
    }

    @Override
    public void runVolts(double volts) {
        pivotMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void resetPosition(double angle) {
        pivotMotor.setPosition(Units.degreesToRotations(angle));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        pivotMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double P, double I, double D) {
        talonConfig.Slot0.kP = P;
        talonConfig.Slot0.kI = I;
        talonConfig.Slot0.kD = D;
        pivotMotor.getConfigurator().apply(talonConfig);
    }

    @Override
    public void setFF(double kA, double kG, double kS, double kV) {
        talonConfig.Slot0.kA = kA;
        talonConfig.Slot0.kG = kG;
        talonConfig.Slot0.kS = kS;
        talonConfig.Slot0.kV = kV;
        pivotMotor.getConfigurator().apply(talonConfig);
    }

    @Override
    public void stop() {
        pivotMotor.setControl(new NeutralOut());
    }
}
