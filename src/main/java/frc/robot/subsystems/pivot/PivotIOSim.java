package frc.robot.subsystems.pivot;

import static frc.robot.constants.GlobalConstants.*;
import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    private final SingleJointedArmSim pivotSim;

    private PIDController controller;

    public PivotIOSim() {
        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(2),
                310 / 3,
                SingleJointedArmSim.estimateMOI(0.3, 10),
                .3,
                Units.degreesToRadians(kBottomDegrees),
                Units.degreesToRadians(kTopDegrees),
                false,
                Units.degreesToRadians(kTopDegrees));

        controller = new PIDController(kGains.kP(), kGains.kI(), kGains.kD());
        resetPosition(kTopDegrees);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        pivotSim.update(kLooperDT);
        inputs.position = Units.radiansToDegrees(pivotSim.getAngleRads());
        inputs.velocity = Units.radiansToDegrees(pivotSim.getVelocityRadPerSec());
        inputs.supplyCurrentAmps = pivotSim.getCurrentDrawAmps();
    }

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void runPosition(double degrees) {
        pivotSim.setInputVoltage(controller.calculate(pivotSim.getAngleRads(), Units.degreesToRadians(degrees)));
    }

    @Override
    public void runVolts(double volts) {
        pivotSim.setInputVoltage(volts);
    }

    @Override
    public void setPID(double P, double I, double D) {
        controller.setPID(P, I, D);
    }

    @Override
    public void stop() {
        pivotSim.setInputVoltage(0);
    }

    @Override
    public void resetPosition(double degrees) {
        pivotSim.setState(Units.degreesToRadians(degrees), 0);
    }
}
