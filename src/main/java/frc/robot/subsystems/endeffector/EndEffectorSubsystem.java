// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Centimeter;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.endeffector.EndEffectorIO.EndEffectorIOInputs;

public class EndEffectorSubsystem extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIOInputs inputs = new EndEffectorIOInputs();
  private final CANrange canRange = new CANrange(0, Constants.CanivoreName);

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem(EndEffectorIO io) {
    this.io = io;
    var config = new CANrangeConfiguration();
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    config.ToFParams.UpdateFrequency = 100;

    config.ProximityParams.ProximityThreshold = 0.15;

    canRange.getConfigurator().apply(config);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getSensorDist() {
    return canRange.getDistance().getValue().in(Centimeter);
  }

  public boolean getHasObject() {
    return canRange.getIsDetected().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
