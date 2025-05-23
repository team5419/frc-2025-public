package frc.robot.commands.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.kTopDegrees;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;

public class PivotReset extends SequentialCommandGroup {

    private static Pivot pivot;

    public PivotReset(RobotContainer robot) {
        pivot = robot.getPivot();
        addRequirements(pivot);

        addCommands(
                new InstantCommand(() -> pivot.setCurrentGoal(PivotGoal.HIGH), pivot),
                new WaitCommand(0.1),
                new InstantCommand(() -> pivot.setCurrentGoal(PivotGoal.IDLE), pivot),
                new InstantCommand(() -> pivot.stop()),
                new InstantCommand(() -> pivot.runVolts(1.7)),
                new WaitCommand(0.1),
                new WaitUntilCommand(() -> (Math.abs(pivot.getMotorVelocity()) <= 0.01)), // ! broken
                new InstantCommand(() -> pivot.runVolts(0)),
                new WaitCommand(0.05),
                new InstantCommand(() -> pivot.zero(kTopDegrees)),
                new InstantCommand(() -> pivot.setCurrentGoal(PivotGoal.HIGH), pivot));
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
