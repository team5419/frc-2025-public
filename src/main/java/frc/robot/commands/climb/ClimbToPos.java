package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbGoal;
import java.util.function.Supplier;

public class ClimbToPos extends Command {

    private Climb climb;
    private Supplier<ClimbGoal> goal;

    public ClimbToPos(Climb c, Supplier<ClimbGoal> goal) {
        this.climb = c;
        this.goal = goal;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setCurrentGoal(goal.get());
    }

    @Override
    public void execute() {
        if (DriverStation.getMatchTime() > 140) {
            climb.setProfileSpeed(false);
        }
    }

    @Override
    public boolean isFinished() {
        return climb.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        climb.setProfileSpeed(goal.get() == ClimbGoal.CAGE_INTAKE);
    }
}
