package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.RobotType;
import frc.robot.lib.DelayedBoolean;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.EnumSet;
import java.util.Set;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private ElevatorIO io;

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/Gains/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/Gains/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/Gains/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/Gains/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/Gains/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/Gains/kA", kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/Gains/kG", kGains.kG());

    private static final LoggedTunableNumber stowHeight = new LoggedTunableNumber("Elevator/Stow Height", kStowHeight);
    private static final LoggedTunableNumber l2 = new LoggedTunableNumber("Elevator/L2", 1.6); // 1.49
    private static final LoggedTunableNumber l3 = new LoggedTunableNumber("Elevator/L3", 3.4); // 3.34
    private static final LoggedTunableNumber l4 =
            new LoggedTunableNumber("Elevator/L4", 6.73); // 6.7 at sac, 6.75 at start of ebr
    private static final LoggedTunableNumber intake =
            new LoggedTunableNumber("Elevator/Intake", 0.32); // was 0.35 at sac, 0.45 at the start of EBR

    private static LoggedTunableNumber voltage = new LoggedTunableNumber("Elevator/Characterization Voltage", 1.0);

    private boolean brakeModeEnabled = false;

    private boolean isChacterization = false;
    private DelayedBoolean isStopped = null;

    public enum ElevatorGoal {
        IDLE(() -> 0), // Should be the current height
        STOW(stowHeight),
        INTAKE(intake),
        INTAKE_FAR(stowHeight),
        L1(stowHeight),
        L2(l2),
        L3(l3),
        L4(l4);

        @Getter
        private DoubleSupplier eleHeight;

        private ElevatorGoal(DoubleSupplier eleHeight) {
            this.eleHeight = eleHeight;
        }
    }

    private Set<ElevatorGoal> lowGoals = EnumSet.of(
            ElevatorGoal.STOW, ElevatorGoal.IDLE, ElevatorGoal.INTAKE_FAR, ElevatorGoal.INTAKE, ElevatorGoal.L1);
    private Set<ElevatorGoal> highGoals = EnumSet.of(ElevatorGoal.L2, ElevatorGoal.L3, ElevatorGoal.L4);
    private Set<ElevatorGoal> scoringGoals =
            EnumSet.of(ElevatorGoal.L1, ElevatorGoal.L2, ElevatorGoal.L3, ElevatorGoal.L4);

    @Getter
    @Setter
    @AutoLogOutput(key = "Elevator/Current Goal")
    private ElevatorGoal currentGoal = ElevatorGoal.STOW;

    @Getter
    @AutoLogOutput(key = "Elevator/Desired Level")
    private ElevatorGoal desiredLevel = ElevatorGoal.L4;

    public Elevator(ElevatorIO io) {
        this.io = io;
        Logger.recordOutput("Elevator/Has reset pos", false);
        setBrakeMode(false);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setFF(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble()),
                kS,
                kG,
                kV,
                kA);

        if (isStopped == null) isStopped = new DelayedBoolean(Timer.getFPGATimestamp(), 0.2);
        if (DriverStation.isDisabled()) {
            stop();
        } else if (!isChacterization && !brakeModeEnabled) {
            if (currentGoal == ElevatorGoal.IDLE) {
                stop();
                return;
            }
            if (currentGoal == ElevatorGoal.STOW && inputs.position[0] <= 0.3) {
                io.runVolts(kStowVolts);
                if (Math.abs(getVelocityLeft()) <= kIsStopped && Math.abs(getVelocityRight()) <= kIsStopped) {
                    isStopped.update(Timer.getFPGATimestamp(), true);
                    stop();
                    io.resetPosition(kStowHeight);
                    Logger.recordOutput("Elevator/Has reset pos", true);
                    isStopped = null;
                    stop();
                }
            } else {
                Logger.recordOutput("Elevator/Has reset pos", false);
                Logger.recordOutput("Elevator/Is stopped", false);
                io.runPosition(currentGoal.eleHeight.getAsDouble(), 0); // FF handled by falcon
            }
        }
    }

    public void resetPosition() {
        io.resetPosition(0);
        Logger.recordOutput("Elevator/Has reset pos", true);
        isStopped = null;
        stop();
    }

    public void stop() {
        setCurrentGoal(ElevatorGoal.IDLE);
        io.stop();
        Logger.recordOutput("Elevator/Is stopped", true);
    }

    public void runVolts(double v) {
        io.stop();
        io.runVolts(v);
    }

    public void setDesiredLevel(ElevatorGoal level) {
        RobotState.getInstance().setEleDesiring(true);
        RobotState.getInstance().setElevatorLevel(level);
        RobotState.getInstance().setDesiredElePos(level);
        desiredLevel = level;
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public void runCharacterization() {
        isChacterization = true;
        io.runVolts(voltage.getAsDouble());
    }

    public void endCharacterization() {
        isChacterization = false;
    }

    public void requestStow() {
        setCurrentGoal(ElevatorGoal.STOW);
    }

    @AutoLogOutput(key = "Elevator/Position")
    public double getPosition() {
        return inputs.position[0];
    }

    public double getVelocityLeft() {
        return inputs.velocityRotationsPerSecond[0];
    }

    public double getVelocityRight() {
        return inputs.velocityRotationsPerSecond[1];
    }

    public double getAvgVelocity() {
        return (getVelocityLeft() + getVelocityRight()) / 2;
    }

    public boolean isRaised() {
        return EqualsUtil.epsilonEquals(
                getPosition(), ElevatorGoal.STOW.eleHeight.getAsDouble(), kPositionTolerance * 2);
    }

    @AutoLogOutput(key = "Elevator/Is Goal Scoring")
    public boolean isGoalScoring() {
        return scoringGoals.contains(currentGoal);
    }

    @AutoLogOutput(key = "Elevator/Is Goal High")
    public boolean isGoalHigh() {
        return highGoals.contains(currentGoal);
    }

    public boolean isGoalHigh(ElevatorGoal goal) {
        return highGoals.contains(goal);
    }

    @AutoLogOutput(key = "Elevator/Will Cross Bar")
    public boolean willCrossBar(ElevatorGoal target) {
        return (highGoals.contains(currentGoal) && lowGoals.contains(target))
                || (highGoals.contains(target) && lowGoals.contains(currentGoal));
    }

    @AutoLogOutput(key = "Elevator/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(getPosition(), currentGoal.eleHeight.getAsDouble(), kPositionTolerance)
                || atSimGoal();
    }

    @AutoLogOutput(key = "Elevator/At Sim Goal")
    public boolean atSimGoal() {
        return GlobalConstants.getRobotType() == RobotType.SIMBOT
                && EqualsUtil.epsilonEquals(
                        getPosition(), Units.feetToMeters(currentGoal.eleHeight.getAsDouble()), kPositionTolerance);
    }
}
