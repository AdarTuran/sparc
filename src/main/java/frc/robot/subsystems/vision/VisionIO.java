package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    public boolean frontCamSeesTarget;
    public boolean backCamSeesTarget;

    public int frontCamBestTarget;

    public MegatagPoseEstimate frontCamPoseEstimate;
    public MegatagPoseEstimate backCamPoseEstimate;

    public Pose2d frontFieldToRobotEstimate;
    public Pose2d backFieldToRobotEstimate;

    public int frontCamTagCount;
    public int backCamTagCount;
  }

  void readInputs(VisionIOInputs inputs);

  void addVisionMeasurement(Pose2d pose, double stdDevs, double timestamp);
}
