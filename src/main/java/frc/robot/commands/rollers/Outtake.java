package frc.robot.commands.rollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.Roller.RollerGoal;
import java.util.function.BooleanSupplier;

public class Outtake extends Command {

    private final Roller roller;
    private final Pivot pivot;
    private final Timer totalOuttakeTimer, beamBreakExitDelay;
    private final BooleanSupplier isL4;

    public Outtake(RobotContainer robot, BooleanSupplier isL4) {
        roller = robot.getRoller();
        pivot = robot.getPivot();
        addRequirements(roller);
        totalOuttakeTimer = new Timer();
        beamBreakExitDelay = new Timer();
        this.isL4 = isL4;
    }

    @Override
    public void initialize() {
        totalOuttakeTimer.restart();
        beamBreakExitDelay.restart();
        roller.setCurrentGoal(RollerGoal.OUTTAKE);
    }

    @Override
    public void execute() {
        pivot.setCurrentGoal(isL4.getAsBoolean() ? PivotGoal.L4 : PivotGoal.SHOOT);
        if (roller.hasCoral()) beamBreakExitDelay.reset();
    }

    @Override
    public void end(boolean isFinished) {
        roller.setCurrentGoal(RollerGoal.IDLE);
        if (RobotState.getInstance().isEleDesiring()) RobotState.getInstance().setEleDesiring(false);
        totalOuttakeTimer.stop();
        beamBreakExitDelay.stop();
    }

    @Override
    public boolean isFinished() {
        return switch (GlobalConstants.getRobotType()) {
            case RADIUM_PROTO -> true;
            default -> beamBreakExitDelay.hasElapsed(0.1)
                    || (DriverStation.isAutonomous() && totalOuttakeTimer.hasElapsed(0.5));
        };
    }
}
