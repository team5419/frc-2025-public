package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.EnumSet;
import java.util.Set;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {

    private RollerIO io;
    private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    private DigitalInput coralBeambreak = new DigitalInput(Ports.kBeamBreakPort);

    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kGains.kS(), kGains.kV(), kGains.kA());

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Roller/kP", kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Roller/kI", kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Roller/kD", kGains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Roller/kS", kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Roller/kV", kGains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Roller/kA", kGains.kA());

    private static final LoggedTunableNumber intake = new LoggedTunableNumber("Roller/Intake", -80.0);
    private static final LoggedTunableNumber gentleIntake = new LoggedTunableNumber("Roller/Gentle Intake", -30);
    private static final LoggedTunableNumber outtake = new LoggedTunableNumber("Roller/Outtake", 20.0);
    private static final LoggedTunableNumber l4Outtake = new LoggedTunableNumber("Roller/L4 Outtake", 10.0);
    private static final LoggedTunableNumber l1Outtake = new LoggedTunableNumber("Roller/L1 Outtake", 10);

    private Timer simBeamBreak;
    private boolean simCoralDetected = true;

    public enum RollerGoal {
        IDLE(() -> 0),
        INTAKE(intake),
        GENTLE_INTAKE(gentleIntake),
        OUTTAKE(outtake),
        L1_OUTTAKE(l1Outtake),
        L4_OUTTAKE(l4Outtake);

        @Getter
        private DoubleSupplier rollVelocity;

        private RollerGoal(DoubleSupplier rollVelocity) {
            this.rollVelocity = rollVelocity;
        }
    }

    @Getter
    @Setter
    @AutoLogOutput(key = "Roller/Current Goal")
    private RollerGoal currentGoal = RollerGoal.IDLE;

    private Set<RollerGoal> outtakeGoal = EnumSet.of(RollerGoal.OUTTAKE, RollerGoal.L1_OUTTAKE, RollerGoal.L4_OUTTAKE);

    public Roller(RollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Roller", inputs);

        // Check controllers
        LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(), sva -> ff = new SimpleMotorFeedforward(sva[0], sva[1], sva[2]), kS, kV, kA);

        if (DriverStation.isDisabled()) {
            setCurrentGoal(RollerGoal.IDLE);
        }

        RobotState.getInstance().setHasCoral(hasCoral());

        // if (hasCoral() && currentGoal != RollerGoal.OUTTAKE && currentGoal != RollerGoal.L4_OUTTAKE)
        //     setCurrentGoal(RollerGoal.GENTLE_INTAKE);
        if (currentGoal == RollerGoal.IDLE) stop();
        else runVelocity(currentGoal.rollVelocity.getAsDouble());
    }

    @AutoLogOutput(key = "Roller/Beambreak Has Coral")
    public boolean hasCoral() {
        return hasCoralSim() || !coralBeambreak.get();
    }

    @AutoLogOutput(key = "Roller/Beam Break Has Coral Sim")
    public boolean hasCoralSim() {
        if (GlobalConstants.getRobotType() != GlobalConstants.RobotType.SIMBOT) return false;
        boolean isIntakeGoal = currentGoal == RollerGoal.INTAKE;
        boolean isOuttakeGoal = outtakeGoal.contains(currentGoal);

        if ((isIntakeGoal && !simCoralDetected || isOuttakeGoal) && simBeamBreak == null) {
            simBeamBreak = new Timer();
            simBeamBreak.start();
        }

        if (simBeamBreak != null && simBeamBreak.hasElapsed(kSimBreamBreakDelay)) {
            simCoralDetected = isIntakeGoal;
            simBeamBreak.stop();
            simBeamBreak = null;
        }

        return simCoralDetected;
    }

    @AutoLogOutput(key = "Roller/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
                inputs.motorVelocityRPS, currentGoal.getRollVelocity().getAsDouble(), kVelocityTolerance);
    }

    public void runVelocity(double velocity) {
        io.runVelocity(velocity, ff.calculate(velocity));
    }

    public void stop() {
        currentGoal = RollerGoal.IDLE;
        io.stop();
    }
}
