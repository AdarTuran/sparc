package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

public class AutoRoutine extends SequentialCommandGroup {
  public AutoRoutine(Drive drive, ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
    addCommands(
        new InstantCommand(() -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), drive),
        new WaitCommand(2.0),
        new InstantCommand(() -> drive.stop(), drive),
        new InstantCommand(() -> elevator.setDistance(16.32), elevator),
        new WaitCommand(1.0),
        new InstantCommand(() -> endEffector.setVoltage(5.5), endEffector),
        new WaitCommand(0.5),
        new InstantCommand(() -> endEffector.setVoltage(0), endEffector));
  }
}
