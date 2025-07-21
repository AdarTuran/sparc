// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algPivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

/** Add your docs here. */
public class AlgPivotIOFalcon implements AlgPivotIO {
  private final TalonFX pivotMotor = new TalonFX(21, Constants.CanivoreName);

  public AlgPivotIOFalcon() {
    configPivotTalonFX(pivotMotor);
    pivotMotor.setPosition(0);
  }

  public void configPivotTalonFX(TalonFX talon) {

    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 25.0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.0;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = 15.75;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Voltage.PeakForwardVoltage = 2.0;
    config.Voltage.PeakReverseVoltage = -2.0;

    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = 20;
    config.MotionMagic.MotionMagicAcceleration = 20;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -127.5 / 360.0;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    talon.getConfigurator().apply(config);
  }

  @Override
  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return pivotMotor.getPosition().getValueAsDouble() * 360.0;
  }

  @Override
  public void setPositionMotionMagic(double targetAngleDeg) {
    pivotMotor.setControl(new MotionMagicVoltage(targetAngleDeg / 360.0));
  }
}
