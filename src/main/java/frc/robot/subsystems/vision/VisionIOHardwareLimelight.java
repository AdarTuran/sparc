package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.concurrent.atomic.AtomicReference;

public class VisionIOHardwareLimelight implements VisionIO {
  NetworkTable frontTable =
      NetworkTableInstance.getDefault().getTable(VisionConstants.kFrontLimelightTableName);

  AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());
  private final Drive drive;

  public VisionIOHardwareLimelight(Drive drive) {
    this.drive = drive;
    setLLSettings();
  }

  private void setLLSettings() {
    Pose2d currentPose = drive.getPose();
    double currentAngularVel = Units.radiansToDegrees(drive.getYawVelocityRadPerSec());

    LimelightHelpers.SetRobotOrientation(
        VisionConstants.kFrontLimelightTableName,
        currentPose.getRotation().getDegrees(),
        currentAngularVel,
        0,
        0,
        0,
        0);

    LimelightHelpers.SetIMUMode(VisionConstants.kFrontLimelightTableName, 1);
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    inputs.frontCamSeesTarget = frontTable.getEntry("tv").getDouble(0) == 1.0;
    if (inputs.frontCamSeesTarget) {
      // Read megatag updates
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              VisionConstants.kFrontLimelightTableName);
      if (megatag2 != null) {
        inputs.frontCamTagCount = megatag2.tagCount;
        inputs.frontCamPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
      }

      inputs.frontCamBestTarget =
          (int) LimelightHelpers.getFiducialID(VisionConstants.kFrontLimelightTableName);
    }
    latestInputs.set(inputs);
    // Set the persistent settings into NT
    if (!RobotState.isDisabled()) {
      setLLSettings();
    }
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double stdDevs, double timestamp) {
    drive.addVisionMeasurement(
        pose, timestamp, VecBuilder.fill(stdDevs, stdDevs, Double.POSITIVE_INFINITY));
  }
}
