// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOKraken implements ElevatorIO {
  private final TalonFX elevatorMotor = new TalonFX(11, Constants.CanivoreName);
  private final TalonFX elevatorMotor2 = new TalonFX(10, Constants.CanivoreName);
  private double targetDistance = 0.0;

  public ElevatorIOKraken() {
    configElevatorKraken(elevatorMotor);
    configElevatorKraken2(elevatorMotor2);

    elevatorMotor.setPosition(0);
    elevatorMotor2.setPosition(0);
  }

  public void configElevatorKraken(TalonFX kraken) {
    kraken.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = 2.5 * (8.0 / 5.0);
    config.Slot0.kD = 0.0;
    config.Slot0.kG = 0.4;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.RotorToSensorRatio = 8.0;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = 150 * (8.0 / 5.0);
    config.MotionMagic.MotionMagicAcceleration = 90 * (8.0 / 5.0);

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 36.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.15;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    kraken.getConfigurator().apply(config);
  }

  // 1.273 * Math.PI inch per rotation
  public void configElevatorKraken2(TalonFX kraken) {
    kraken.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    config.Slot0.kP = 2.5 * (8.0 / 5.0);
    config.Slot0.kD = 0.0;
    config.Slot0.kG = 0.4;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.RotorToSensorRatio = 8.0;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = 150 * (8.0 / 5.0);
    config.MotionMagic.MotionMagicAcceleration = 90 * (8.0 / 5.0);

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 36.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.15;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    kraken.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    inputs.elevatorDistance = getDistance();
    inputs.elevatorCurrentAmps = getElevatorCurrent();
  }

  @Override
  public void setElevatorDistance(double distance) {
    targetDistance = distance;
    elevatorMotor.setControl(new MotionMagicVoltage(distance));
    elevatorMotor2.setControl(new Follower(elevatorMotor.getDeviceID(), true));
  }

  @Override
  public void setElevatorVolts(double voltage) {
    elevatorMotor.setVoltage(voltage);
    elevatorMotor2.setControl(new Follower(elevatorMotor.getDeviceID(), true));
  }

  @Override
  public double getDistance() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getElevatorCurrent() {
    return elevatorMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public double getTargetDistance() {
    return targetDistance;
  }
}
