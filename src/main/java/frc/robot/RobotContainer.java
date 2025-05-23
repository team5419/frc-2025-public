// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ExitableCommand;
import frc.robot.commands.climb.ClimbToPos;
import frc.robot.commands.elevator.SafeElevate;
import frc.robot.commands.elevator.StowReset;
import frc.robot.commands.pivot.PivotToPos;
import frc.robot.commands.rollers.IntakeSequential;
import frc.robot.commands.rollers.Outtake;
import frc.robot.commands.swerve.AutoAlignToCoral;
import frc.robot.commands.swerve.DisplaceAlgae;
import frc.robot.commands.swerve.DriveCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import frc.robot.lib.RumbleThread;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhoton;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbGoal;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.Roller.RollerGoal;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;
import java.io.File;
import java.util.function.BooleanSupplier;
import lombok.Getter;

public class RobotContainer {

    @Getter
    private CommandXboxController driver = new CommandXboxController(Ports.kDriverPort);

    @Getter
    private CommandXboxController operator = new CommandXboxController(Ports.kOperatorPort);

    @Getter
    private Swerve swerve;

    @Getter
    private Roller roller;

    @Getter
    private Elevator elevator;

    @Getter
    private Pivot pivot;

    @Getter
    private Climb climb;

    @Getter
    private final SendableChooser<Command> autoChooser;

    private BooleanSupplier driverOverride;

    @Getter
    AprilTagVision aprilTagVision;

    public RobotContainer() {
        driverOverride = () -> Math.hypot(driver.getLeftX(), driver.getLeftY()) > GlobalConstants.kOverrideJoystick
                || Math.abs(driver.getRightX()) > GlobalConstants.kOverrideJoystick;
        // Get driver station to stop
        DriverStation.silenceJoystickConnectionWarning(true);

        buildRobot();
        configNamedCommands();

        autoChooser = buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureDefaultCommands();
        if (GlobalConstants.kDevMode) {
            configureDevBindings();
        } else {
            configureDriverBindings();
            configureOperatorBindings();
        }

        RumbleThread.getInstance().bindControllers(driver, operator);
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(DriveCommands.joystickDrive(
                swerve,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> driver.leftBumper().getAsBoolean()));
    }

    private void configureDriverBindings() {
        driver.start().onTrue(Commands.runOnce(() -> swerve.setPose(FieldConstants.kFrontReefPose))); // odometry reset
        driver.back().onTrue(new ClimbToPos(climb, () -> ClimbGoal.STOW)); // stow the climber

        driver.a() // stow elevator
                .onTrue(new StowReset(this)
                        .andThen(new InstantCommand(() -> elevator.setCurrentGoal(ElevatorGoal.INTAKE), elevator)));

        driver.b() // climb
                .onTrue(Commands.parallel(
                        new ClimbToPos(climb, () -> ClimbGoal.CLIMB),
                        new SafeElevate(this, () -> ElevatorGoal.STOW)
                                .andThen(new PivotToPos(this, () -> PivotGoal.HOLD_LOW))));
        driver.x() // emergency shoot in case of beambreak fail
                .onTrue(new InstantCommand(() -> roller.setCurrentGoal(RollerGoal.OUTTAKE)))
                .onFalse(new InstantCommand(() -> roller.setCurrentGoal(RollerGoal.IDLE)));
        driver.y().onTrue(Commands.runOnce(() -> swerve.resetGyro()).ignoringDisable(true)); // reset gyro angle

        driver.povUp(); // ! Unbound
        driver.povLeft(); // ! Unbound
        driver.povRight(); // ! Unbound
        driver.povDown(); // ! Unbound

        driver.leftBumper(); // ! SWERVE SLOW MODE
        driver.rightBumper()
                .onTrue(new ConditionalCommand(
                        new ExitableCommand(new DisplaceAlgae(this, driver), driverOverride),
                        new AutoScore(this, driver),
                        operator.b()::getAsBoolean));
        // auto align & outtake if at goal

        driver.leftTrigger(0.1) // run rollers in
                .whileTrue(new InstantCommand(() -> roller.setCurrentGoal(RollerGoal.INTAKE)))
                .onFalse(new InstantCommand(() -> roller.setCurrentGoal(RollerGoal.IDLE)));

        driver.rightTrigger(0.1) // elevate and shoot coral
                .whileTrue(new SafeElevate(this, () -> elevator.getDesiredLevel(), false))
                .onFalse(new Outtake(this, () -> elevator.getDesiredLevel() == ElevatorGoal.L4));
    }

    private void configureOperatorBindings() {
        operator.start() // emergency pivot reset to low
                .onTrue(new InstantCommand(() -> pivot.zero(PivotConstants.kBottomDegrees)));
        operator.back().onTrue(new InstantCommand(() -> pivot.stop())); // emergency pivot run zero volts

        operator.a().whileTrue(new IntakeSequential(this, ElevatorGoal.INTAKE_FAR));
        operator.b().onChange(new InstantCommand(() -> RobotState.getInstance()
                .setDisplacing(operator.b().getAsBoolean())));
        operator.x(); // ! Unbound
        operator.y(); // ! Unbound

        operator.povUp() // set elevator goal level to L4
                .onTrue(new InstantCommand(() -> elevator.setDesiredLevel(ElevatorGoal.L4)));
        operator.povDown() // set elevator goal level to L1
                .onTrue(new InstantCommand(() -> elevator.setDesiredLevel(ElevatorGoal.L1)));
        operator.povLeft() // set elevator goal level to L2
                .onTrue(new InstantCommand(() -> elevator.setDesiredLevel(ElevatorGoal.L2)));
        operator.povRight() // set elevator goal level to L3
                .onTrue(new InstantCommand(() -> elevator.setDesiredLevel(ElevatorGoal.L3)));

        operator.leftBumper() // set scoring location to an earlier branch
                .onTrue(Commands.runOnce(() -> RobotState.getInstance().setEarly(false)));
        operator.rightBumper() // set scoring location to a later branch
                .onTrue(Commands.runOnce(() -> RobotState.getInstance().setEarly(true)));

        // operator.leftTrigger(0.1)
        // .onTrue(new IntakeSequential(this, ElevatorGoal.INTAKE)
        // .alongWith(new RotateToSource(
        // this, driver, () -> driver.leftBumper().getAsBoolean()))); // intake & move
        // pivot

        operator.leftTrigger(0.1).whileTrue(new IntakeSequential(this, ElevatorGoal.INTAKE));

        operator.rightTrigger(0.1) // prep climb (extend down)
                .onTrue(new ParallelCommandGroup(
                        new ClimbToPos(climb, () -> ClimbGoal.CAGE_INTAKE), new StowReset(this)));
    }

    /**
     * Use this function to test new features without
     * changing the current button bindings
     */
    private void configureDevBindings() {
        operator.start();
        operator.back();

        operator.a()
                .whileTrue(new InstantCommand(() -> climb.runVolts(4)))
                .onFalse(new InstantCommand(() -> climb.runVolts(0)));
        operator.b()
                .whileTrue(new InstantCommand(() -> climb.runVolts(-4)))
                .onFalse(new InstantCommand(() -> climb.runVolts(0)));
        operator.x().onTrue(new InstantCommand(() -> climb.setStopped(true)));
        operator.y().onTrue(new InstantCommand(() -> climb.zero(ClimbConstants.kStowAngle)));

        operator.povUp().onTrue(new InstantCommand(() -> pivot.setCurrentGoal(PivotGoal.HIGH)));
        operator.povDown();
        operator.povLeft();
        operator.povDown().onTrue(new InstantCommand(() -> pivot.setCurrentGoal(PivotGoal.LOW)));
    }

    private void buildRobot() {

        Swerve tempSwerve = null;
        Roller tempRoller = null;
        Elevator tempElevator = null;
        Pivot tempPivot = null;
        Climb tempClimb = null;

        if (GlobalConstants.getMode() == GlobalConstants.Mode.REPLAY) return;
        switch (GlobalConstants.getRobotType()) {
            case ORBIT -> {
                tempSwerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackRight()));
            }

            case RADIUM_PROTO -> {
                tempSwerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackRight()));
            }

            case ALPHA -> {
                tempClimb = new Climb(new ClimbIOTalonFX());
                tempSwerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackRight()));
                tempPivot = new Pivot(new PivotIOTalonFX());
                tempRoller = new Roller(new RollerIOTalonFX());
                tempElevator = new Elevator(new ElevatorIOTalonFX());
                aprilTagVision = new AprilTagVision(this, new AprilTagVisionIOPhoton());
            }

            case BETA -> {
                // pivot = new Pivot(new PivotIOTalonFX());
                // tempClimb = new Climb(new ClimbIOTalonFX());
                tempSwerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackRight()));
            }

            case SIMBOT -> {
                tempRoller = new Roller(new RollerIOSim());
                tempElevator = new Elevator(new ElevatorIOSim());
                tempPivot = new Pivot(new PivotIOSim());
                tempSwerve = new Swerve(
                        new GyroIO() {},
                        new ModuleIOSim(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getBackRight()));
            }
        }

        if (tempSwerve == null)
            tempSwerve = new Swerve(
                    new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});

        if (tempRoller == null) tempRoller = new Roller(new RollerIO() {});

        if (tempElevator == null) tempElevator = new Elevator(new ElevatorIO() {});

        if (tempPivot == null) tempPivot = new Pivot(new PivotIO() {});

        if (tempClimb == null) tempClimb = new Climb(new ClimbIO() {});

        swerve = tempSwerve;
        roller = tempRoller;
        elevator = tempElevator;
        pivot = tempPivot;
        climb = tempClimb;
    }

    /** Adds named commands to pathplanner */
    private void configNamedCommands() {

        NamedCommands.registerCommand("Intake", new IntakeSequential(this, ElevatorGoal.INTAKE));

        NamedCommands.registerCommand("elevate to L4", new SafeElevate(this, () -> ElevatorGoal.L4));

        NamedCommands.registerCommand("Outtake", new Outtake(this, () -> true)); // true bc we only score L4 in auto

        NamedCommands.registerCommand("Stow", new SafeElevate(this, () -> ElevatorGoal.STOW));

        NamedCommands.registerCommand(
                "Reset Odometry Left",
                new InstantCommand(() ->
                        swerve.setPose(AllianceFlipUtil.apply(new Pose2d(7.23, 6.35, Rotation2d.fromDegrees(-160))))));

        // sets early to true then auto aligns
        ParallelRaceGroup autoAlignEarly = new ParallelRaceGroup(
                new SequentialCommandGroup(
                        new InstantCommand(() -> RobotState.getInstance().setEarly(true)),
                        new AutoAlignToCoral(this, driver)),
                new WaitCommand(4));
        NamedCommands.registerCommand("Auto-Align Early", autoAlignEarly);

        // sets early to false then auto aligns
        ParallelRaceGroup autoAlignLate = new ParallelRaceGroup(
                new SequentialCommandGroup(
                        new InstantCommand(() -> RobotState.getInstance().setEarly(false)),
                        new AutoAlignToCoral(this, driver)),
                new WaitCommand(4));
        NamedCommands.registerCommand("Auto-Align Late", autoAlignLate);

        NamedCommands.registerCommand(
                "Record Time", new InstantCommand(() -> RobotState.getInstance().setAutoFinished(true)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do nothing", new InstantCommand());
        File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
        for (File f : autosDir.listFiles()) {
            if (!f.isDirectory()) {
                String fileName[] = f.getName().split("\\.");
                String autoName = fileName[0];
                chooser.addOption(
                        autoName, AutoBuilder.buildAuto(autoName).beforeStarting(() -> System.out.println(autoName)));
            }
        }
        // PathPlannerAuto rightSide = new PathPlannerAuto("3-Coral-Left-Odometry", true);
        chooser.addOption("3-Coral-Right-Odometry", new PathPlannerAuto("3-Coral-Left-Odometry", true));
        return chooser;
    }
}
