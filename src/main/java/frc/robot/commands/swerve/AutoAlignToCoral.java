package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class AutoAlignToCoral extends Command {
    private static final double LINEARTOL = 0.07;
    private static final double THETATOL = 0.07;
    private static final double LINEARDEADBAND = 0.03;
    private static final double THETADEADBAND = 0.03;
    private static final double TOLTIMEOUT = 0.1;
    private static final double ANGLE_MAX_VELOCITY = Math.PI * 4;
    private static final double ANGLE_MAX_ACCELERATION = Math.PI * 4;
    private static final double LINEAR_MAX_ACCELERATION = 7;
    private static final double CONTROLLER_EXIT = 0.8;

    private static final double MAX_DISTANCE_TO_CORAL = 4.0;
    private static final double DISTANCE_WAIT_FOR_ELE = 0.8;
    private static double MAX_SPEED_SCALAR = 0.8;
    private static double LINEAR_MAX_VELOCITY;

    private static final LoggedTunableNumber linearkP = new LoggedTunableNumber("AutoAlign/drivekP", 0.8);
    private static final LoggedTunableNumber linearkD = new LoggedTunableNumber("AutoAlign/drivekD", 0.05);
    private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("AutoAlign/thetakP", 0.2);
    private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("AutoAlign/thetakD", 0.0);

    private static final LoggedTunableNumber linearTolorance =
            new LoggedTunableNumber("AutoAlign/Linear Tolorance", LINEARTOL);
    private static final LoggedTunableNumber thetaTolorance =
            new LoggedTunableNumber("AutoAlign/Theta Tolorance", THETATOL);

    private final ProfiledPIDController linearXController;
    private final ProfiledPIDController linearYController;
    private final ProfiledPIDController thetaController;
    private final Timer tolTimer;
    private final DoubleSupplier leftX;
    private final DoubleSupplier leftY;
    private Pose2d targetPose;
    private Swerve swerve;
    private Elevator elevator;
    private CommandXboxController driver;
    private double startingJoystickValue;

    private boolean joystickWentDown;

    public AutoAlignToCoral(RobotContainer robot, CommandXboxController driver) {
        this.driver = driver;
        this.leftX = () -> driver.getLeftX();
        this.leftY = () -> driver.getLeftY();
        elevator = robot.getElevator();
        swerve = robot.getSwerve();
        addRequirements(swerve);
        LINEAR_MAX_VELOCITY = swerve.getMaxLinearSpeedMetersPerSec() * MAX_SPEED_SCALAR;

        linearXController = new ProfiledPIDController(
                linearkP.get(),
                0,
                linearkD.get(),
                new TrapezoidProfile.Constraints(swerve.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));

        linearYController = new ProfiledPIDController(
                linearkP.get(),
                0,
                linearkD.get(),
                new TrapezoidProfile.Constraints(LINEAR_MAX_VELOCITY, LINEAR_MAX_ACCELERATION));

        thetaController = new ProfiledPIDController(
                thetakP.get(),
                0,
                thetakD.get(),
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        tolTimer = new Timer();

        setTolorances(linearTolorance.get(), thetaTolorance.get());

        updateConstraints();
        resetControllers();
    }

    @Override
    public void initialize() {

        RobotState.getInstance().setAutoAlignAtGoal(false);

        if (DriverStation.isAutonomous() && !swerve.getIsOnLeftHalf())
            RobotState.getInstance().setEarly(!RobotState.getInstance().isEarly());

        targetPose = swerve.getBestCoralAutoAlign();

        if (PhotonUtils.getDistanceToPose(swerve.getPose(), targetPose) > MAX_DISTANCE_TO_CORAL) {
            this.cancel();
            return;
        }

        startingJoystickValue = Math.hypot(leftX.getAsDouble(), leftY.getAsDouble());
        joystickWentDown = false;

        swerve.setIsAutoAligning(true);
        resetControllers();
        RobotState.getInstance().setAligning(true);
    }

    @Override
    public void execute() {

        // updateConstraints(); ! ADD THIS BACK

        // LoggedTunableNumber.ifChanged(
        // hashCode(), pid -> updatePIDS(pid[0], pid[1], pid[2], pid[3]), linearkP, linearkD, thetakP, thetakD);

        // LoggedTunableNumber.ifChanged(
        // hashCode(), tolorances -> setTolorances(tolorances[0], tolorances[1]), linearTolorance, thetaTolorance);

        Pose2d currentPose = swerve.getPose();

        double driveVelocityX = MathUtil.applyDeadband(
                linearXController.calculate(currentPose.getX(), targetPose.getX())
                        * swerve.getMaxLinearSpeedMetersPerSec(),
                LINEARDEADBAND);
        double driveVelocityY = MathUtil.applyDeadband(
                linearYController.calculate(currentPose.getY(), targetPose.getY())
                        * swerve.getMaxLinearSpeedMetersPerSec(),
                LINEARDEADBAND);
        double thetaVelocity = MathUtil.applyDeadband(
                thetaController.calculate(
                                currentPose.getRotation().getRadians(),
                                targetPose.getRotation().getRadians())
                        * swerve.getMaxAngularSpeedRadPerSec(),
                THETADEADBAND);
        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocityX, driveVelocityY, thetaVelocity, swerve.getPose().getRotation());

        double scalar = 1;
        if (PhotonUtils.getDistanceToPose(currentPose, targetPose) < DISTANCE_WAIT_FOR_ELE
                && (elevator.getPosition() < ElevatorGoal.L3.getEleHeight().getAsDouble()
                        && elevator.getDesiredLevel() == ElevatorGoal.L4)) {
            scalar = 0.25;
        } else if (PhotonUtils.getDistanceToPose(currentPose, targetPose) < DISTANCE_WAIT_FOR_ELE
                && (elevator.getDesiredLevel() == ElevatorGoal.L4)) {
            scalar = 0.8;
        }
        swerve.runVelocity(adjustedSpeeds.times(scalar));

        if (!linearXController.atGoal() || !linearYController.atGoal() || !thetaController.atGoal()) {
            tolTimer.restart();
        }

        if (Math.hypot(leftX.getAsDouble(), leftY.getAsDouble()) < Math.max(startingJoystickValue - 0.2, 0.2))
            joystickWentDown = true;

        RobotState.getInstance().setAutoAlignAtGoal(tolTimer.hasElapsed(TOLTIMEOUT));

        Logger.recordOutput("AutoAlign/Joystick Went Down", joystickWentDown);

        // Logger.recordOutput("AutoAlign/Auto-Align Active", true);
        Logger.recordOutput("AutoAlign/LinearX At Goal", linearXController.atGoal());
        Logger.recordOutput("AutoAlign/LinearY At Goal", linearYController.atGoal());
        Logger.recordOutput("AutoAlign/Theta At Goal", thetaController.atGoal());
        // Logger.recordOutput("AutoAlign/DriveVelocityX", driveVelocityX);
        // Logger.recordOutput("AutoAlign/DriveVelocityY", driveVelocityY);
        // Logger.recordOutput("AutoAlign/ThetaVelocity", thetaVelocity);
        // Logger.recordOutput("AutoAlign/TargetPose", targetPose);
        // Logger.recordOutput("AutoAlign/Tol Timer", tolTimer.get());
    }

    @Override
    public boolean isFinished() {
        return (joystickWentDown
                        && (Math.abs(driver.getLeftX()) > CONTROLLER_EXIT
                                || Math.abs(driver.getLeftY()) > CONTROLLER_EXIT
                                || Math.abs(driver.getRightX()) > CONTROLLER_EXIT))
                || tolTimer.hasElapsed(TOLTIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("AutoAlign/Auto-Align Active", false);
        RobotState.getInstance().setAligning(false);
        swerve.setIsAutoAligning(false);
        Logger.recordOutput(
                "AutoAlign/Offset Transform When Aligned",
                swerve.getBestReefTagNoOffset().minus(swerve.getPose()));
    }

    @SuppressWarnings("unused")
    private void updatePIDS(double linearP, double linearD, double thetaP, double thetaD) {
        linearXController.setPID(linearP, 0, linearD);
        linearYController.setPID(linearP, 0, linearD);
        thetaController.setPID(thetaP, 0, thetaD);
    }

    private void setTolorances(double linear, double theta) {
        linearXController.setTolerance(linear);
        linearYController.setTolerance(linear);
        thetaController.setTolerance(theta);
    }

    private void updateConstraints() {
        double maxSpeed = LINEAR_MAX_VELOCITY;
        linearXController.setConstraints(new TrapezoidProfile.Constraints(maxSpeed, LINEAR_MAX_ACCELERATION));
        linearYController.setConstraints(new TrapezoidProfile.Constraints(maxSpeed, LINEAR_MAX_ACCELERATION));
        thetaController.setConstraints(new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    }

    private void resetControllers() {
        Pose2d currentPose = swerve.getPose();
        ChassisSpeeds fieldVelocity = swerve.getChassisSpeeds();

        linearXController.reset(currentPose.getX(), fieldVelocity.vxMetersPerSecond);
        linearYController.reset(currentPose.getY(), fieldVelocity.vyMetersPerSecond);
        thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    }
}
