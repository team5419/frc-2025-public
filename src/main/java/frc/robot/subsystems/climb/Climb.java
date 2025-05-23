package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

    private ClimbIO io;

    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private final Alert climbBroken = new Alert("Not running climb bc of position delta!", AlertType.kWarning);
    private final Alert climbDiscon = new Alert("One or more climb motors disconnected!", AlertType.kWarning);

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/Gains/kP", ClimbConstants.kGains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Climb/Gains/kI", ClimbConstants.kGains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/Gains/kD", ClimbConstants.kGains.kD());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Climb/Gains/kA", ClimbConstants.kGains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Climb/Gains/kG", ClimbConstants.kGains.kG());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Climb/Gains/kS", ClimbConstants.kGains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Climb/Gains/kV", ClimbConstants.kGains.kV());

    private static final LoggedTunableNumber stowGoal =
            new LoggedTunableNumber("Climb/Stow Angle", ClimbConstants.kStowAngle);
    private static final LoggedTunableNumber algaeIntakeGoal = new LoggedTunableNumber("Climb/Algae Intake Angle", 60);
    private static final LoggedTunableNumber cageIntakeGoal = new LoggedTunableNumber("Climb/Cage Intake Angle", 121);
    private static final LoggedTunableNumber climbPosition = new LoggedTunableNumber("Climb/Climb Angle", -16); // -14.5

    public enum ClimbGoal {
        IDLE(() -> 0),
        STOW(stowGoal),
        ALGAE_INTAKE(algaeIntakeGoal),
        CAGE_INTAKE(cageIntakeGoal),
        CLIMB(climbPosition);

        private DoubleSupplier angle;

        private ClimbGoal(DoubleSupplier angle) {
            this.angle = angle;
        }
    }

    @Setter
    @AutoLogOutput(key = "Climb/Is stopped")
    private boolean stopped = false;

    @AutoLogOutput(key = "Climb/Is Running volts")
    private boolean runVolts = false;

    @Getter
    @AutoLogOutput(key = "Climb/Current Goal")
    private ClimbGoal currentGoal = ClimbGoal.STOW;

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setFF(kA.get(), kG.get(), kS.get(), kV.get()), kA, kG, kS, kV);
        if (runVolts) return;
        climbBroken.set(Math.abs(inputs.positionDegrees[0] - inputs.positionDegrees[1]) > kDeltaThreshold);
        climbDiscon.set(!inputs.leaderMotorConnected || !inputs.followerMotorConnected);
        if (Math.abs(inputs.positionDegrees[0] - inputs.positionDegrees[1]) > kDeltaThreshold) stopped = true;
        if (atGoal() && currentGoal != ClimbGoal.CLIMB) stopped = true;
        if (currentGoal == ClimbGoal.CLIMB && atGoal()) RobotState.getInstance().setClimbing(true);
        if (currentGoal != ClimbGoal.CLIMB) RobotState.getInstance().setClimbing(false);
        if (!stopped) io.runPosition(currentGoal.angle.getAsDouble());
        else io.stop();
    }

    public void runVolts(double volts) {
        runVolts = true;
        currentGoal = ClimbGoal.IDLE;
        io.runVolts(volts);
    }

    public void setCurrentGoal(ClimbGoal goal) {
        runVolts = false;
        currentGoal = goal;
        stopped = false;
    }

    public void zero(double angle) {
        io.resetPosition(angle);
    }

    public void setProfileSpeed(boolean slow) {
        io.setProfile(slow);
    }

    @AutoLogOutput(key = "Climb/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(inputs.positionDegrees[0], currentGoal.angle.getAsDouble(), kAngleTolerance);
    }
}
