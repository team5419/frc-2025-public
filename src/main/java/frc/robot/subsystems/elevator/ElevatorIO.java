package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs {
        public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;

        public double[] position = new double[] {0, 0};
        public double[] velocityRotationsPerSecond = new double[] {0, 0};

        public double[] referencePosition = new double[] {0, 0};
        public double[] referenceVelocity = new double[] {0, 0};
        public double[] referenceError = new double[] {0, 0};

        public double[] appliedVolts = new double[] {0, 0};

        public double[] supplyCurrentAmps = new double[] {0, 0};
        public double[] statorCurrentAmps = new double[] {0, 0};
        public double[] tempCelsius = new double[] {0, 0};
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void runPosition(double eleHeight, double feedforward) {}

    default void resetPosition(double pos) {}

    default void runVolts(double volts) {}

    default void runVelocity(double velocity) {}

    default void stop() {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setFF(double kS, double kG, double kV, double kA) {}
}
