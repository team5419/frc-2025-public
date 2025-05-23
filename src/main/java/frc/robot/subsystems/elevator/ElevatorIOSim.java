package frc.robot.subsystems.elevator;

import static frc.robot.constants.GlobalConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.AutoLogOutput;

public class ElevatorIOSim implements ElevatorIO {

    private final PIDController controller;

    @AutoLogOutput(key = "Elevator/Sim/AppliedVolts")
    private double appliedVolts = 0.0;

    private final ElevatorSim eSim;

    public ElevatorIOSim() {
        controller = new PIDController(kGains.kP(), kGains.kI(), kGains.kD());
        eSim = new ElevatorSim(DCMotor.getKrakenX60(2), 10, 10, .1, 0, 7, false, 0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        eSim.update(kLooperDT);

        final double dummyMotorTemp = 42.0;
        final double current = eSim.getCurrentDrawAmps() / 2.0;

        inputs.position = new double[] {eSim.getPositionMeters(), eSim.getPositionMeters()};
        inputs.velocityRotationsPerSecond =
                new double[] {eSim.getVelocityMetersPerSecond(), eSim.getVelocityMetersPerSecond()};
        inputs.appliedVolts = new double[] {appliedVolts, appliedVolts};
        inputs.tempCelsius = new double[] {dummyMotorTemp, dummyMotorTemp};
        inputs.supplyCurrentAmps = new double[] {current, current};
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        eSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void runPosition(double setpointMeters, double feedforward) {
        if (DriverStation.isDisabled()) {
            return;
        }
        runVolts(controller.calculate(eSim.getPositionMeters(), Units.feetToMeters(setpointMeters)) + feedforward);
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
