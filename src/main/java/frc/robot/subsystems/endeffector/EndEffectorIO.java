// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {}

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
