package frc.robot.subsystems.climb;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimbIOInputsAutoLogged extends ClimbIO.ClimbIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeaderMotorConnected", leaderMotorConnected);
    table.put("FollowerMotorConnected", followerMotorConnected);
    table.put("PositionDegrees", positionDegrees);
    table.put("VelocityMetersPerSecond", velocityMetersPerSecond);
    table.put("AppliedVolts", appliedVolts);
    table.put("SupplyCurrentAmps", supplyCurrentAmps);
    table.put("StatorCurrentAmps", statorCurrentAmps);
    table.put("TempCelsius", tempCelsius);
    table.put("PositionReference", positionReference);
    table.put("MotorReferenceVelocity", motorReferenceVelocity);
  }

  @Override
  public void fromLog(LogTable table) {
    leaderMotorConnected = table.get("LeaderMotorConnected", leaderMotorConnected);
    followerMotorConnected = table.get("FollowerMotorConnected", followerMotorConnected);
    positionDegrees = table.get("PositionDegrees", positionDegrees);
    velocityMetersPerSecond = table.get("VelocityMetersPerSecond", velocityMetersPerSecond);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
    statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
    tempCelsius = table.get("TempCelsius", tempCelsius);
    positionReference = table.get("PositionReference", positionReference);
    motorReferenceVelocity = table.get("MotorReferenceVelocity", motorReferenceVelocity);
  }

  public ClimbIOInputsAutoLogged clone() {
    ClimbIOInputsAutoLogged copy = new ClimbIOInputsAutoLogged();
    copy.leaderMotorConnected = this.leaderMotorConnected;
    copy.followerMotorConnected = this.followerMotorConnected;
    copy.positionDegrees = this.positionDegrees.clone();
    copy.velocityMetersPerSecond = this.velocityMetersPerSecond.clone();
    copy.appliedVolts = this.appliedVolts.clone();
    copy.supplyCurrentAmps = this.supplyCurrentAmps.clone();
    copy.statorCurrentAmps = this.statorCurrentAmps.clone();
    copy.tempCelsius = this.tempCelsius.clone();
    copy.positionReference = this.positionReference.clone();
    copy.motorReferenceVelocity = this.motorReferenceVelocity.clone();
    return copy;
  }
}
