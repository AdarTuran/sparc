// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim sim =
      new ElevatorSim(DCMotor.getKrakenX60Foc(2), 5, 9, 0.03, 0.2, 5, true, 0.2, 0.0, 0.0);

  private final Mechanism2d m_mech2d = new Mechanism2d(20, 30);

  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 1);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(new MechanismLigament2d("Elevator", sim.getPositionMeters(), 90));

  private PIDController pid = new PIDController(50.0, 0, 0);

  private double targetDistance = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged io) {
    sim.update(0.02);
    m_elevatorMech2d.setLength(sim.getPositionMeters());
    SmartDashboard.putData("ElevatorSim", m_mech2d);
  }

  @Override
  public void setElevatorDistance(double distance) {
    targetDistance = distance;
    var volts = pid.calculate(sim.getPositionMeters(), distance);
    sim.setInputVoltage(volts);
  }

  @Override
  public void setElevatorVolts(double voltage) {
    sim.setInputVoltage(voltage);
  }

  @Override
  public double getDistance() {
    return sim.getPositionMeters();
  }

  @Override
  public double getTargetDistance() {
    return targetDistance;
  }
}
