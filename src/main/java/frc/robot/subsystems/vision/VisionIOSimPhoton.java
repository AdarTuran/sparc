package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSimPhoton extends VisionIOHardwareLimelight {

  private final PhotonCamera frontCamera = new PhotonCamera("frontCamera");
  private final PhotonCamera backCamera = new PhotonCamera("backCamera");

  PhotonCameraSim frontCameraSim;
  PhotonCameraSim backCameraSim;

  private final VisionSystemSim visionSim;
  private final int kResWidth = 1280;
  private final int kResHeight = 800;

  private final Drive drive;
  private final Transform3d robotToFrontCamera =
      new Transform3d(
          VisionConstants.kFrontLimelightTranslation, VisionConstants.kFrontLimelightRotation);
  private final Transform3d robotToBackCamera =
      new Transform3d(
          VisionConstants.kBackLimelightTranslation, VisionConstants.kBackLimelightRotation);

  public VisionIOSimPhoton(Drive drive) {
    super(drive);
    this.drive = drive;
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(VisionConstants.kAprilTagLayout);

    // Create simulated camera properties. These can be set to mimic your actual
    // camera.
    var frontProp = new SimCameraProperties();
    frontProp.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
    frontProp.setCalibError(0.35, 0.10);
    frontProp.setFPS(30);
    frontProp.setAvgLatencyMs(20);
    frontProp.setLatencyStdDevMs(5);

    var backProp = new SimCameraProperties();
    backProp = frontProp.copy();

    frontCameraSim = new PhotonCameraSim(frontCamera, frontProp);
    backCameraSim = new PhotonCameraSim(backCamera, backProp);

    visionSim.addCamera(frontCameraSim, robotToFrontCamera);
    visionSim.addCamera(backCameraSim, robotToBackCamera);

    // Enable the raw and processed streams. (http://localhost:1181 / 1182)
    frontCameraSim.enableRawStream(true);
    frontCameraSim.enableProcessedStream(true);
    backCameraSim.enableRawStream(true);
    backCameraSim.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    frontCameraSim.enableDrawWireframe(true);
    backCameraSim.enableDrawWireframe(true);
  }

  public void readInputs(VisionIOInputs inputs) {
    Pose2d estimatedPose = drive.getPose();
    if (estimatedPose != null) {
      visionSim.update(estimatedPose);
    }
    Logger.recordOutput("Vision/SimIO/updateSimPose", estimatedPose);

    NetworkTable frontTable =
        NetworkTableInstance.getDefault().getTable(VisionConstants.kFrontLimelightTableName);
    NetworkTable backTable =
        NetworkTableInstance.getDefault().getTable(VisionConstants.kBackLimelightTableName);

    var frontCamResults = frontCamera.getAllUnreadResults();
    var backCamResults = backCamera.getAllUnreadResults();
    if (frontCamResults.size() > 0) {
      writeToTable(
          frontCamResults.get(frontCamResults.size() - 1),
          frontTable,
          new Transform3d(
              new Translation3d().minus(VisionConstants.kFrontLimelightTranslation),
              new Rotation3d().minus(VisionConstants.kFrontLimelightRotation)));
    }
    if (backCamResults.size() > 0) {
      writeToTable(
          backCamResults.get(backCamResults.size() - 1),
          backTable,
          new Transform3d(
              VisionConstants.kBackLimelightTranslation,
              new Rotation3d().minus(VisionConstants.kBackLimelightRotation)));
    }

    super.readInputs(inputs);
  }

  private void writeToTable(PhotonPipelineResult result, NetworkTable table, Transform3d camPose) {
    // Write to limelight table
    if (result.getMultiTagResult().isPresent()) {
      Transform3d best = result.getMultiTagResult().get().estimatedPose.best;
      best = best.plus(camPose);
      Pose2d fieldToCamera =
          new Pose2d(best.getTranslation().toTranslation2d(), best.getRotation().toRotation2d());
      List<Double> pose_data =
          new ArrayList<>(
              Arrays.asList(
                  best.getX(), // 0: X
                  best.getY(), // 1: Y
                  best.getZ(), // 2: Z,
                  0.0, // 3: roll
                  0.0, // 4: pitch
                  fieldToCamera.getRotation().getDegrees(), // 5: yaw
                  result.metadata.getLatencyMillis(), // 6: latency ms,
                  (double) result.getMultiTagResult().get().fiducialIDsUsed.size(), // 7: tag count
                  0.0, // 8: tag span
                  result
                      .getBestTarget()
                      .bestCameraToTarget
                      .getTranslation()
                      .getNorm(), // 9: tag dist
                  result.getBestTarget().getArea() // 10: tag area
                  ));
      // Add RawFiducials
      // This is super inefficient but it's sim only, who cares.
      for (var target : result.targets) {
        pose_data.add((double) target.getFiducialId()); // 0: id
        pose_data.add(target.getYaw()); // 1: txnc
        pose_data.add(target.getPitch()); // 2: tync
        pose_data.add(0.0); // 3: ta
        pose_data.add(0.0); // 4: distToCamera
        pose_data.add(0.0); // 5: distToRobot
        pose_data.add(0.5); // 6: ambiguity
      }

      table
          .getEntry("botpose_wpiblue")
          .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
      table
          .getEntry("botpose_orb_wpiblue")
          .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
    }

    if (result.hasTargets()) {
      Optional<PhotonTrackedTarget> closestTarget =
          result.getTargets().stream().max((r1, r2) -> Double.compare(r1.area, r2.area));
      if (closestTarget.isPresent()) {
        var target = closestTarget.get();
        table.getEntry("tx").setDouble(target.yaw);
        table.getEntry("ty").setDouble(target.pitch);
        table.getEntry("txnc").setDouble(target.yaw);
        table.getEntry("tync").setDouble(target.pitch);
        table.getEntry("tid").setDouble(target.fiducialId);
      }
    }

    table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
    table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
  }
}
