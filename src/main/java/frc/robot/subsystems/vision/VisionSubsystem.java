// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private double lastProcessedFrontTimestamp = 0.0;
  private double lastProcessedBackTimestamp = 0.0;

  public static boolean allowVisionUpdates = false;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO io) {
    this.io = io;
    SmartDashboard.putString(
        "AAAAXXXDD", VisionConstants.kAprilTagLayout.getTagPose(7).get().toPose2d().toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double timestamp = (double) Logger.getTimestamp() * 1.0E-6;
    io.readInputs(inputs);
    Logger.processInputs("Vision", inputs);

    updateVision(inputs.frontCamSeesTarget, inputs.frontCamPoseEstimate, true);

    Logger.recordOutput(
        "Vision/latencyPeriodicSec", ((double) Logger.getTimestamp() * 1.0E-6) - timestamp);
  }

  private void updateVision(
      boolean cameraSeesTarget,
      MegatagPoseEstimate cameraMegatag2PoseEstimate,
      boolean isFrontCam) {
    if (cameraMegatag2PoseEstimate != null) {
      var updateTimestamp = cameraMegatag2PoseEstimate.timestampSeconds;
      boolean alreadyProcessedTimestamp =
          (isFrontCam ? lastProcessedFrontTimestamp : lastProcessedBackTimestamp)
              == updateTimestamp;

      if (!alreadyProcessedTimestamp && cameraSeesTarget) {
        final double xyStdDev = calculateCameraStdDev(cameraMegatag2PoseEstimate);

        if (xyStdDev == Double.POSITIVE_INFINITY) {
          return;
        }

        if (Double.isNaN(cameraMegatag2PoseEstimate.robotToField.getX())
            || Double.isNaN(cameraMegatag2PoseEstimate.robotToField.getY())
            || Double.isInfinite(cameraMegatag2PoseEstimate.robotToField.getX())
            || Double.isInfinite(cameraMegatag2PoseEstimate.robotToField.getY())) {
          return;
        }

        if (Constants.currentMode != Mode.SIM && !RobotState.isDisabled() && allowVisionUpdates) {
          io.addVisionMeasurement(
              cameraMegatag2PoseEstimate.robotToField, xyStdDev, updateTimestamp);
        }
      }

      if (isFrontCam) {
        lastProcessedFrontTimestamp = updateTimestamp;
      } else {
        lastProcessedBackTimestamp = updateTimestamp;
      }
    }
  }

  public Pose2d getAlignmentPoseWithOffset(Translation2d offset, int targetID) {
    Rotation2d targetAngle =
        Rotation2d.fromDegrees(
            VisionConstants.kAprilTagLayout
                .getTagPose(targetID)
                .get()
                .getRotation()
                .getMeasureZ()
                .in(Degrees));

    offset =
        offset.plus(
            new Translation2d(
                (Constants.robotLengthMeters / 2.0) - Constants.bumperWidthMeters, 0.0));

    Translation2d targetTranslation =
        VisionConstants.tagList
            .get(targetID - 1)
            .pose
            .toPose2d()
            .transformBy(new Transform2d(offset, new Rotation2d()))
            .getTranslation();

    targetAngle = targetAngle.rotateBy(Rotation2d.fromDegrees(180));

    return new Pose2d(targetTranslation, targetAngle);
  }

  private double calculateCameraStdDev(MegatagPoseEstimate poseEstimate) {
    final double xyStdDevCoefficient = 0.08;
    double xyStdDev = Double.POSITIVE_INFINITY;
    final int tagCount = poseEstimate.fiducialIds.length;
    final double avgTagDist = poseEstimate.avgTagDist;

    if (avgTagDist > 0 && tagCount > 0) {
      xyStdDev = xyStdDevCoefficient * Math.pow(avgTagDist, 2.0) / tagCount;
    }

    return xyStdDev;
  }

  public int getCurrentReefTagID() {
    return inputs.frontCamBestTarget;
  }

  public boolean getFrontCamTargetValid() {
    return inputs.frontCamSeesTarget;
  }
}
