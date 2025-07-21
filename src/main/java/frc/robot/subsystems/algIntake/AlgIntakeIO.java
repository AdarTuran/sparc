// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algIntake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface AlgIntakeIO {
  @AutoLog
  public static class AlgIntakeIOInputs {}

  public default void updateInputs(AlgIntakeIOInputs io) {}

  public default void setVoltage(double voltage) {}
}
