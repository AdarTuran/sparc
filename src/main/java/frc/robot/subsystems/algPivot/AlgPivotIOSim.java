// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algPivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class AlgPivotIOSim implements AlgPivotIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          100,
          SingleJointedArmSim.estimateMOI(1, 5),
          1,
          0,
          180 * Math.PI / 180,
          false,
          0);
  private PIDController pid = new PIDController(0.2, 0.0, 0.0);

  private final Mechanism2d m_mech2d = new Mechanism2d(0.60, 0.60);
  private final MechanismRoot2d m_pivotPivot = m_mech2d.getRoot("pivotPivot", 0.30, 0.30);
  private final MechanismLigament2d m_pivotTower =
      m_pivotPivot.append(new MechanismLigament2d("pivotTower", 1, -90));
  private final MechanismLigament2d m_pivot =
      m_pivotPivot.append(
          new MechanismLigament2d(
              "pivot",
              1,
              Units.radiansToDegrees(sim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  private double volts = 0.0;
  private double targetAngle = 0.0;

  @Override
  public void updateInputs(AlgPivotIOInputs inputs) {
    sim.update(0.02);
    m_pivot.setAngle(Math.toDegrees(sim.getAngleRads()));
    SmartDashboard.putData("pivot Sim", m_mech2d);
  }

  @Override
  public void setPivotAngle(double angle) {
    targetAngle = angle;
    volts = pid.calculate(Math.toDegrees(sim.getAngleRads()), angle);
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
  }

  @Override
  public double getPivotAngle() {
    return -(sim.getAngleRads() - Math.toRadians(28));
  }

  @Override
  public double getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void stopPivot() {
    sim.setInputVoltage(0);
  }
}
