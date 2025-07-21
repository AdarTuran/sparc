// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Add your docs here. */
public class ElevatorCommands {

  private ElevatorCommands() {}

  public static Command setDistance(ElevatorSubsystem elevator, double distance) {
    return Commands.runEnd(
        () -> {
          elevator.setDistance(distance);
        },
        () -> {
          if (distance != 0.0) {
            elevator.setElevatorVolts(0.32);
          } else {
            elevator.setElevatorVolts(0);
          }
        },
        elevator);
  }

  public static Command setElevatorVolts(ElevatorSubsystem elevator, double voltage) {
    return Commands.runEnd(
        () -> {
          elevator.setElevatorVolts(voltage);
        },
        () -> elevator.setElevatorVolts(0.32),
        elevator);
  }
}
