// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final ElevatorIO io;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  public static boolean atL4 = false;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  public void setDistance(double distance) {
    io.setElevatorDistance(distance);
  }

  public void setElevatorVolts(double voltage) {
    io.setElevatorVolts(voltage);
  }

  public double getDistance() {
    return io.getDistance();
  }

  public double getTargetDistance() {
    return io.getTargetDistance();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    if (getDistance() >= 17.0) {
      atL4 = true;
    } else {
      atL4 = false;
    }
  }
}
