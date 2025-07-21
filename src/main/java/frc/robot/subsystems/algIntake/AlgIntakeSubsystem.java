// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgIntakeSubsystem extends SubsystemBase {
  private AlgIntakeIO io;
  private AlgIntakeIOInputsAutoLogged inputs;
  /** Creates a new AlgIntakeIOSubsystem. */
  public AlgIntakeSubsystem(AlgIntakeIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // This method will be called once per scheduler run
  }
}
