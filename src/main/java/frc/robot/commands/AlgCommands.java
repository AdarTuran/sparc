// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algIntake.AlgIntakeSubsystem;
import frc.robot.subsystems.algPivot.AlgPivotSubsystem;

/** Add your docs here. */
// () ->
public class AlgCommands {
  public static Command setPivotVoltage(AlgPivotSubsystem pivot, double voltage) {
    return Commands.runEnd(() -> pivot.setVoltage(voltage), () -> pivot.setVoltage(0.0), pivot);
  }

  public static Command setPivotPosition(AlgPivotSubsystem pivot, double pos) {
    return Commands.runEnd(
        () -> pivot.setPositionMotionMagic(pos), () -> pivot.setVoltage(0.0), pivot);
  }

  public static Command setIntakeVoltage(AlgIntakeSubsystem intake, double voltage) {
    return Commands.runEnd(() -> intake.setVoltage(voltage), () -> intake.setVoltage(0), intake);
  }
}
