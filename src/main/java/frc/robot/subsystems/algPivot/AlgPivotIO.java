// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algPivot;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface AlgPivotIO {
  @AutoLog
  public static class AlgPivotIOInputs {}

  public default void updateInputs(AlgPivotIOInputs io) {}

  public default void setPivotAngle(double angle) {}

  public default double getPivotAngle() {
    return 0.0;
  }

  public default double getTargetAngle() {
    return 0.0;
  }

  public default void stopPivot() {}

  public default void setVoltage(double voltage) {}

  public default double getPosition() {
    return 0;
  }

  public default void setPositionMotionMagic(double targetAngleDeg) {}
}
