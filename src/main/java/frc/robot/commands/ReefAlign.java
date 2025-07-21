// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefAlign extends Command {
  /** Creates a new ReefAlign. */
  private final VisionSubsystem vision;

  private final Drive drive;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(1.35 * 2.3, 0, 0, new TrapezoidProfile.Constraints(2.0, 1.0));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(2.5 * 2.5, 0, 0, new TrapezoidProfile.Constraints(3.0, 1.0));

  private ProfiledPIDController angleController =
      new ProfiledPIDController(6.0, 0.0, 0.1, new TrapezoidProfile.Constraints(8.0, 20.0));

  private Pose2d targetPose;
  private int currentTarget;
  private boolean alignLeft;

  public ReefAlign(VisionSubsystem vision, Drive drive, boolean alignLeft) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.drive = drive;
    this.alignLeft = alignLeft;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);

    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.reset(drive.getRotation().getRadians());
    angleController.setTolerance(Math.toRadians(0.5));

    if (vision.getFrontCamTargetValid()) {
      currentTarget = vision.getCurrentReefTagID();
      targetPose =
          vision.getAlignmentPoseWithOffset(
              alignLeft ? new Translation2d(0.00, -0.165) : new Translation2d(0.00, 0.165),
              currentTarget);

      xController.reset(0);
      yController.reset(0);
      Logger.recordOutput("PoseToAlign", targetPose);
    } else {
      currentTarget = -1;
    }
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    // Get target rotation in radians
    double targetTheta = targetPose.getRotation().getRadians();

    // Compute target unit vectors
    double normX = Math.cos(targetTheta);
    double normY = Math.sin(targetTheta);

    // Compute field-relative position error
    double errorX = targetPose.getX() - currentPose.getX();
    double errorY = targetPose.getY() - currentPose.getY();

    // Transform errors into normal & tangential components
    double errorNormal = errorX * normX + errorY * normY;
    double errorTangent = -errorX * normY + errorY * normX;

    // Apply separate PIDs for normal & tangent motion
    double vNormal = xController.calculate(errorNormal);
    double vTangent = yController.calculate(errorTangent);

    // Convert normal/tangent velocities back to field-relative XY
    double xVel = vNormal * normX - vTangent * normY;
    double yVel = vNormal * normY + vTangent * normX;

    // Flip velocity for red alliance if necessary
    xVel = -xVel;
    yVel = -yVel;

    if (!yController.atSetpoint() && Math.abs(errorNormal) <= 0.15) {
      xVel = xVel * 0.25;
    }

    // Convert to robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVel,
            yVel,
            angleController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()),
            drive.getRotation());

    // Run the robot with corrected robot-relative speeds
    drive.runOpenLoop(robotRelativeSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentTarget == -1;
  }
}
