// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.funnel.FunnelSubsystem;

/** Add your docs here. */
public class FunnelCommands {
  // () ->
  public static Command setFunnelVoltage(FunnelSubsystem funnel, double voltage) {
    return Commands.runEnd(() -> funnel.setVoltage(voltage), () -> funnel.setVoltage(0), funnel);
  }
}
