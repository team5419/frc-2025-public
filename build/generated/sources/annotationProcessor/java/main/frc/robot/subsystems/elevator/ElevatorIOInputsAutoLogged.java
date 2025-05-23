package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeaderMotorConnected", leaderMotorConnected);
    table.put("FollowerMotorConnected", followerMotorConnected);
    table.put("Position", position);
    table.put("VelocityRotationsPerSecond", velocityRotationsPerSecond);
    table.put("ReferencePosition", referencePosition);
    table.put("ReferenceVelocity", referenceVelocity);
    table.put("ReferenceError", referenceError);
    table.put("AppliedVolts", appliedVolts);
    table.put("SupplyCurrentAmps", supplyCurrentAmps);
    table.put("StatorCurrentAmps", statorCurrentAmps);
    table.put("TempCelsius", tempCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    leaderMotorConnected = table.get("LeaderMotorConnected", leaderMotorConnected);
    followerMotorConnected = table.get("FollowerMotorConnected", followerMotorConnected);
    position = table.get("Position", position);
    velocityRotationsPerSecond = table.get("VelocityRotationsPerSecond", velocityRotationsPerSecond);
    referencePosition = table.get("ReferencePosition", referencePosition);
    referenceVelocity = table.get("ReferenceVelocity", referenceVelocity);
    referenceError = table.get("ReferenceError", referenceError);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
    statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
    tempCelsius = table.get("TempCelsius", tempCelsius);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.leaderMotorConnected = this.leaderMotorConnected;
    copy.followerMotorConnected = this.followerMotorConnected;
    copy.position = this.position.clone();
    copy.velocityRotationsPerSecond = this.velocityRotationsPerSecond.clone();
    copy.referencePosition = this.referencePosition.clone();
    copy.referenceVelocity = this.referenceVelocity.clone();
    copy.referenceError = this.referenceError.clone();
    copy.appliedVolts = this.appliedVolts.clone();
    copy.supplyCurrentAmps = this.supplyCurrentAmps.clone();
    copy.statorCurrentAmps = this.statorCurrentAmps.clone();
    copy.tempCelsius = this.tempCelsius.clone();
    return copy;
  }
}
