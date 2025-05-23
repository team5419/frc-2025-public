package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    class ClimbIOInputs {
        public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;

        public double[] positionDegrees = new double[] {0, 0};
        public double[] velocityMetersPerSecond = new double[] {0, 0};

        public double[] appliedVolts = new double[] {0, 0};

        public double[] supplyCurrentAmps = new double[] {0, 0};
        public double[] statorCurrentAmps = new double[] {0, 0};
        public double[] tempCelsius = new double[] {0, 0};

        public double[] positionReference = new double[] {0, 0};
        public double[] motorReferenceVelocity = new double[] {0, 0};
    }

    default void runPosition(double targetPosition) {}

    default void updateInputs(ClimbIOInputs inputs) {}

    default void runVolts(double volts) {}

    default void setBrakeMode(boolean enabled) {}

    default void resetPosition(double angle) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setFF(double kA, double kG, double kS, double kV) {}

    default void setProfile(boolean slow) {}

    default void stop() {}
}
