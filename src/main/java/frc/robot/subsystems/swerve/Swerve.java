// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.Mode;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.GeomUtil;
import frc.robot.subsystems.swerve.generated.LocalADStarAK;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

    static final Lock odometryLock = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private Rotation2d rawGyroRotation = new Rotation2d();

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private SwerveModulePosition[] lastModulePositions; // For delta tracking
    private SwerveDrivePoseEstimator poseEstimator;

    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SysIdRoutine sysId;

    private boolean isAutoAligning = false;

    public Swerve(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {

        try {
            // Goal -- To override the default configs with the config in the GUI
            SwerveConstants.kPPConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.err.println(" -------------------------------------------------------- ");
            System.err.println("   [SWERVE] [ERR] Unable to make robot config from GUT.   ");
            System.err.println(" -------------------------------------------------------- ");
        }

        // Set our gyro IO
        this.gyroIO = gyroIO;

        // Initialize our last module positions
        lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        // Create pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

        // Create our modules
        modules[0] = new Module(flModuleIO, 0, TunerConstants.getFrontLeft());
        modules[1] = new Module(frModuleIO, 1, TunerConstants.getFrontRight());
        modules[2] = new Module(blModuleIO, 2, TunerConstants.getBackLeft());
        modules[3] = new Module(brModuleIO, 3, TunerConstants.getBackRight());

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                SwerveConstants.kAutoController,
                SwerveConstants.kPPConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);

        Pathfinding.setPathfinder(new LocalADStarAK());

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Autos/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Autos/TrajectorySetpoint", targetPose);
        });

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {

        odometryLock.lock(); // Prevents odometry updates while reading data

        gyroIO.updateInputs(gyroInputs);

        Logger.processInputs("Swerve/Gyro", gyroInputs);
        Logger.recordOutput("Swerve/Pose", getPose());
        Logger.recordOutput("Swerve/Field Pose/Best Pose with offset", getBestCoralAutoAlign());
        Logger.recordOutput(
                "Swerve/Field Pose/Best Source", getBestSourceAutoAlign().getDegrees());
        Logger.recordOutput("Swerve/Field Pose/Robot Angle", getRotation().getDegrees());
        Logger.recordOutput(
                "Swerve/Auto Align/Offset Transform", getBestReefTagNoOffset().minus(getPose()));

        for (var module : modules) {
            module.periodic();
        }

        odometryLock.unlock();

        if (DriverStation.isDisabled()) {

            // Stop moving when disabled
            for (var module : modules) {
                module.stop();
            }

            // Log empty setpoint states when disabled
            Logger.recordOutput("Swerve/SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        updateOdometry();

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && GlobalConstants.getMode() != Mode.SIM);
    }

    private void updateOdometry() {

        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together

        for (int i = 0; i < sampleTimestamps.length; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Swerve/SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void resetGyro() {
        setPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will return to their normal orientations the next time
     * a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities)
     * for all of the modules.
     */
    @AutoLogOutput(key = "Swerve/SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions)
     * for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "Swerve/SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec
     * (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Swerve/Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Swerve/Odometry/Lookahead Pose")
    public Pose2d getFuturePose(double dt) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
        Pose2d robot = getPose();

        double dx = robot.getX() + (speeds.vxMetersPerSecond * dt);
        double dy = robot.getY() + (speeds.vyMetersPerSecond * dt);
        double dTheta = getRotation().getRadians() + (speeds.omegaRadiansPerSecond * dt);

        return new Pose2d(dx, dy, new Rotation2d(dTheta));
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Rotation2d getBestSourceAutoAlign() {
        return getIsOnLeftHalf()
                ? AllianceFlipUtil.apply(FieldConstants.kSourceRotationLeft.toRotation2d())
                        .unaryMinus()
                        .plus(Rotation2d.fromDegrees(180))
                : AllianceFlipUtil.apply(FieldConstants.kSourceRotationRight.toRotation2d())
                        .unaryMinus()
                        .plus(Rotation2d.fromDegrees(180));
    }

    private Pose2d getBestCoralTag() {
        List<Pose2d> poses = new ArrayList<Pose2d>();
        for (Pose3d pose : FieldConstants.CoralTags.kCoralTags) {
            poses.add(AllianceFlipUtil.apply(
                    pose.toPose2d().plus(GeomUtil.toTransform2d(FieldConstants.CoralTags.kReefOffset))));
        }
        return getFuturePose(0.4).nearest(poses);
    }

    /** Gets the pose of the closest coral tag without offset */
    public Pose2d getBestReefTagNoOffset() {
        List<Pose2d> poses = new ArrayList<Pose2d>();
        for (Pose3d pose : FieldConstants.CoralTags.kCoralTags) {
            poses.add(AllianceFlipUtil.apply(pose.toPose2d()));
        }
        return getPose().nearest(poses);
    }

    public Pose2d getBestCoralAutoAlign() {
        Translation2d offset = RobotState.getInstance().isEarly()
                ? FieldConstants.CoralTags.kCoralAlignOffset.plus(
                        new Translation2d(-0.025, 0.05 + .0175 + Units.inchesToMeters(1.5)))
                : FieldConstants.CoralTags.kCoralAlignOffset
                        .plus(new Translation2d(0.025, 0.142))
                        .unaryMinus();
        return getBestCoralTag()
                .plus(GeomUtil.toTransform2d(offset))
                .plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));
    }

    public boolean getIsOnLeftHalf() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? getPose().getY() < FieldConstants.kFieldWidth / 2
                : getPose().getY() >= FieldConstants.kFieldWidth / 2;
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @AutoLogOutput(key = "Swerve/Absolute Field Velocity")
    public double getAbsoluteFieldVelocity() {
        return Math.hypot(
                Math.abs(getChassisSpeeds().vxMetersPerSecond), Math.abs(getChassisSpeeds().vyMetersPerSecond));
    }

    @AutoLogOutput(key = "Swerve/Field Velocity")
    public Twist2d getVelocity() {
        return getChassisSpeeds().toTwist2d(GlobalConstants.kLooperDT);
    }

    @AutoLogOutput(key = "Swerve/Absolute Velocity")
    public double getAbsoluteRobotVelocity() {
        return Math.hypot(
                Math.abs(getChassisSpeeds().vxMetersPerSecond), Math.abs(getChassisSpeeds().vyMetersPerSecond));
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return SwerveConstants.kSpeedAt12Volts;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / kDriveBaseRadius;
    }

    public boolean getIsAutoAligning() {
        return isAutoAligning;
    }

    public void setIsAutoAligning(boolean autoAligning) {
        isAutoAligning = autoAligning;
    }
}
