package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

    private PivotIO io;

    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", PivotConstants.kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", PivotConstants.kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", PivotConstants.kGains.kD());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", PivotConstants.kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", PivotConstants.kGains.kG());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", PivotConstants.kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", PivotConstants.kGains.kV());

    private static final LoggedTunableNumber shootGoal =
            new LoggedTunableNumber("Pivot/Shoot Goal Angle", kZeroDegrees + 2);
    private static final LoggedTunableNumber highGoal =
            new LoggedTunableNumber("Pivot/High goal angle", kTopDegrees + 1);
    private static final LoggedTunableNumber farIntakeGoal =
            new LoggedTunableNumber("Pivot/Far goal angle", kTopDegrees - 5);
    private static final LoggedTunableNumber lowGoal = new LoggedTunableNumber("Pivot/Low goal angle", kBottomDegrees);
    private static final LoggedTunableNumber l1Goal =
            new LoggedTunableNumber("Pivot/L1 Goal Angle", shootGoal.getAsDouble() + 10);
    private static final LoggedTunableNumber l4Goal = new LoggedTunableNumber("Pivot/L4 Goal Angle", kZeroDegrees - 4);

    private double velocityThreshold = 1; // change later

    public enum PivotGoal {
        IDLE(() -> 0),
        HIGH(highGoal),
        LOW(lowGoal),
        SHOOT(shootGoal),
        L1(l1Goal),
        L4(l4Goal),
        FAR_INTAKE(farIntakeGoal),
        HOLD_LOW(lowGoal);

        @Getter
        private DoubleSupplier angle;

        private PivotGoal(DoubleSupplier angle) {
            this.angle = angle;
        }
    }

    @Setter
    @AutoLogOutput(key = "Pivot/Is stopped")
    private boolean stopped = false;

    @Getter
    private double motorAngle = inputs.position;

    @Getter
    private double motorVelocity = inputs.velocity;

    @Getter
    @AutoLogOutput(key = "Pivot/Current Goal")
    private PivotGoal currentGoal = PivotGoal.HIGH;

    public Pivot(PivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        motorAngle = inputs.position;
        motorVelocity = inputs.velocity;
        Logger.processInputs("Pivot", inputs);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setFF(kA.get(), kG.get(), kS.get(), kV.get()), kA, kG, kS, kV);
        if (currentGoal == PivotGoal.LOW && atGoal()) {
            stop();
            // zero(PivotConstants.kTopDegrees);
        } else if (currentGoal == PivotGoal.IDLE) {
            return;
        } else {
            runSetpoint(currentGoal);
        }
    }

    public void stop() {
        stopped = true;
        io.stop();
    }

    public void setVolts(double voltage, double maintainenceVoltage, PivotGoal targetGoal) {
        if (inputs.velocity < velocityThreshold) {
            io.runVolts(targetGoal == PivotGoal.HIGH ? voltage : -voltage);
        } else {
            io.runVolts(maintainenceVoltage);
        }
    }
    // for the PivotReset command
    public void runVolts(double voltage) {
        io.runVolts(voltage);
    }

    private void runSetpoint(PivotGoal goal) {
        io.runPosition(goal.angle.getAsDouble());
    }

    public void setCurrentGoal(PivotGoal goal) {
        if (goal != currentGoal) stopped = false;
        currentGoal = goal;
    }

    public void zero(double angle) {
        io.resetPosition(angle);
    }

    @AutoLogOutput(key = "Pivot/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(inputs.position, currentGoal.angle.getAsDouble(), kAngleTolerance);
    }

    public boolean isSafe() {
        return inputs.position < kPivotSafeToMoveAngle;
    }

    public double getVelocity() {
        return inputs.velocity;
    }
}
