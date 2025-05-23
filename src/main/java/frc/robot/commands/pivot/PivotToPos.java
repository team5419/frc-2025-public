package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.Roller.RollerGoal;
import java.util.function.Supplier;

public class PivotToPos extends Command {

    private final Pivot pivot;
    private final Roller roller;
    private final Supplier<PivotGoal> goal;

    public PivotToPos(RobotContainer robot, Supplier<PivotGoal> goal) {
        pivot = robot.getPivot();
        roller = robot.getRoller();
        this.goal = goal;
        addRequirements(pivot, roller);
    }

    @Override
    public void initialize() {
        roller.setCurrentGoal(RollerGoal.GENTLE_INTAKE);
    }

    @Override
    public void execute() {
        pivot.setCurrentGoal(goal.get());
    }

    @Override
    public boolean isFinished() {
        return pivot.atGoal();
    }

    @Override
    public void end(boolean isFinished) {
        roller.setCurrentGoal(RollerGoal.IDLE);
    }
}
