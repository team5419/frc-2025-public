package frc.robot.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.GlobalConstants;

public class RollerIOSim implements RollerIO {

    private final PIDController controller =
            new PIDController(RollerConstants.kGains.kP(), RollerConstants.kGains.kI(), RollerConstants.kGains.kD());
    private double appliedVolts = 0.0;

    private final DCMotorSim rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(RollerConstants.kGains.kV(), RollerConstants.kGains.kA()),
            new DCMotor(appliedVolts, appliedVolts, appliedVolts, appliedVolts, 0, 1));

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        rollerSim.update(GlobalConstants.kLooperHZ);

        final double dummyMotorTemp = 42.0;
        final double current = rollerSim.getCurrentDrawAmps() / 2.0;

        inputs.motorPositionRads = rollerSim.getAngularPositionRad();
        inputs.motorVelocityRPS = rollerSim.getAngularVelocity().magnitude();
        inputs.motorAppliedVolts = appliedVolts;
        inputs.motorTempCelsius = dummyMotorTemp;
        inputs.motorSupplyCurrentAmps = current;
    }

    @Override
    public void runVelocity(double motorRPS, double ff) {
        rollerSim.setAngularVelocity(Units.rotationsToRadians(motorRPS));
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        rollerSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
