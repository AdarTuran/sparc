// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double elevatorDistance;
    public double elevatorCurrentAmps;
  }

  public default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}

  public default void setElevatorDistance(double distance) {}

  public default void setElevatorVolts(double voltage) {}

  public default double getElevatorCurrent() {
    return 0.0;
  }

  public default double getDistance() {
    return 0.0;
  }

  public default double getTargetDistance() {
    return 0.0;
  }
}
