package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;

public class RollerIOTalonFX implements RollerIO {

    private TalonFX rollerMotor;
    private TalonFXConfiguration config;

    private final VoltageOut reqVoltage = new VoltageOut(0.0).withEnableFOC(true);
    private final NeutralOut reqNeutral = new NeutralOut();
    private final VelocityVoltage reqVelocity = new VelocityVoltage(0.0);

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorTempCelsius;

    public RollerIOTalonFX() {

        rollerMotor = new TalonFX(kRollerConfig.ID(), GlobalConstants.kRIOName);

        motorPosition = rollerMotor.getPosition();
        motorVelocity = rollerMotor.getVelocity();
        motorAppliedVoltage = rollerMotor.getMotorVoltage();
        motorSupplyCurrent = rollerMotor.getSupplyCurrent();
        motorTorqueCurrent = rollerMotor.getTorqueCurrent();
        motorTempCelsius = rollerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorSupplyCurrent,
                motorTorqueCurrent,
                motorTempCelsius);

        config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = 1;

        config.Slot0.kP = kGains.kP();
        config.Slot0.kI = kGains.kI();
        config.Slot0.kD = kGains.kD();

        rollerMotor.getConfigurator().apply(config);
        rollerMotor.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        motorPosition,
                        motorVelocity,
                        motorAppliedVoltage,
                        motorSupplyCurrent,
                        motorTorqueCurrent,
                        motorTempCelsius)
                .isOK();

        inputs.motorPositionRads = motorPosition.getValueAsDouble();
        inputs.motorVelocityRPS = motorVelocity.getValueAsDouble();
        inputs.motorAppliedVolts = motorAppliedVoltage.getValueAsDouble();
        inputs.motorSupplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();
        inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorTempCelsius.getValueAsDouble();
    }

    @Override
    public void runVolts(double motorVolts) {
        rollerMotor.setControl(reqVoltage.withOutput(motorVolts));
    }

    @Override
    public void stop() {
        rollerMotor.setControl(reqNeutral);
    }

    @Override
    public void runVelocity(double motorRPS, double ff) {
        rollerMotor.setControl(reqVelocity.withVelocity(motorRPS).withFeedForward(ff));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        rollerMotor.getConfigurator().apply(config);
    }
}
