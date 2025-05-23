package frc.robot.commands.rollers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.GlobalConstants;
import frc.robot.lib.RumbleThread;
import frc.robot.lib.RumbleThread.ControllersToRumble;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.Roller.RollerGoal;
import java.util.function.Supplier;

public class Intake extends Command {

    private final Roller roller;
    private final Pivot pivot;
    private final Supplier<PivotGoal> goal;

    public Intake(RobotContainer robot, Supplier<PivotGoal> goal) {
        roller = robot.getRoller();
        pivot = robot.getPivot();
        this.goal = goal;
        addRequirements(roller, pivot);
    }

    @Override
    public void initialize() {
        roller.setCurrentGoal(RollerGoal.INTAKE);
        RumbleThread.getInstance().startRumbleIndefinetely(RumbleType.kBothRumble, 0.3, ControllersToRumble.ALL);
    }

    @Override
    public void execute() {
        pivot.setCurrentGoal(goal.get());
    }

    @Override
    public void end(boolean isFinished) {
        roller.setCurrentGoal(RollerGoal.IDLE);
        RumbleThread.getInstance().stopAllRumbles();
    }

    @Override
    public boolean isFinished() {
        return switch (GlobalConstants.getRobotType()) {
            case RADIUM_PROTO -> true;
            default -> roller.hasCoral();
        };
    }
}
