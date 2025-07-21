// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

/** Add your docs here. */
public class EndEffectorCommands {

  public EndEffectorCommands() {}

  public static Command setEndEffectorVolts(EndEffectorSubsystem endEffector, double voltage) {
    return Commands.runEnd(
        () -> endEffector.setVoltage(voltage), () -> endEffector.setVoltage(0), endEffector);
  }

  public static Command endEffectorSensorCommand(EndEffectorSubsystem endEffector, double voltage) {
    return Commands.sequence(
        Commands.waitUntil(endEffector::getHasObject)
            .raceWith(
                Commands.runEnd(
                    () -> endEffector.setVoltage(voltage),
                    () -> endEffector.setVoltage(0),
                    endEffector)),
        EndEffectorCommands.setEndEffectorVolts(endEffector, 1.8).withTimeout(0.0));
  }
}
