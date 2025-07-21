// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {
  private Servo servo = new Servo(9);
  private FunnelIO io;
  private FunnelIOInputsAutoLogged inputs;
  /** Creates a new FunnelIOSubsystem. */
  public FunnelSubsystem(FunnelIO io) {
    this.io = io;
  }

  public void setServoHome() {
    servo.set(1);
  }

  public void setServoDisable() {
    servo.setDisabled();
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
