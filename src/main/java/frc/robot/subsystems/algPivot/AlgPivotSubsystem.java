// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algPivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class AlgPivotSubsystem extends SubsystemBase {
  private AlgPivotIO io;
  private AlgPivotIOInputsAutoLogged inputs;
  /** Creates a new AlgPivotIOSubsystem. */
  public AlgPivotSubsystem(AlgPivotIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @AutoLogOutput(key = "AlgPivot/Position")
  public double getPosition() {
    return io.getPosition();
  }

  public void setPositionMotionMagic(double targetDeg) {
    io.setPositionMotionMagic(targetDeg);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // This method will be called once per scheduler run
  }
}
