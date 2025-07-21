package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class MegatagPoseEstimate implements StructSerializable {
  public static final Pose2d kPose2dZero = new Pose2d();

  public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {
    public Pose2d robotToField = kPose2dZero;
    public double timestampSeconds;
    public double latency;
    public double avgTagDist;

    @Override
    public Class<MegatagPoseEstimate> getTypeClass() {
      return MegatagPoseEstimate.class;
    }

    @Override
    public String getTypeString() {
      return "struct:MegatagPoseEstimate";
    }

    @Override
    public int getSize() {
      return Pose2d.struct.getSize() + kSizeDouble * 3;
    }

    @Override
    public String getSchema() {
      return "Pose2d robotToField;double timestampSeconds;double latency;double avgTagDist;double tagCount;";
    }

    @Override
    public Struct<?>[] getNested() {
      return new Struct<?>[] {Pose2d.struct};
    }

    @Override
    public MegatagPoseEstimate unpack(ByteBuffer bb) {
      MegatagPoseEstimate rv = new MegatagPoseEstimate();
      rv.robotToField = Pose2d.struct.unpack(bb);
      rv.timestampSeconds = bb.getDouble();
      rv.latency = bb.getDouble();
      rv.avgTagDist = bb.getDouble();
      rv.fiducialIds = new int[0];
      return rv;
    }

    @Override
    public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
      Pose2d.struct.pack(bb, value.robotToField);
      bb.putDouble(value.timestampSeconds);
      bb.putDouble(value.latency);
      bb.putDouble(value.avgTagDist);
    }

    @Override
    public String getTypeName() {
      return "MegatagPoseEstimateStruct";
    }
  }

  public Pose2d robotToField = kPose2dZero;
  public double timestampSeconds;
  public double latency;
  public double avgTagDist;
  public int[] fiducialIds;

  public MegatagPoseEstimate() {}

  public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
    MegatagPoseEstimate rv = new MegatagPoseEstimate();
    rv.robotToField = poseEstimate.pose;
    if (rv.robotToField == null) rv.robotToField = kPose2dZero;
    rv.timestampSeconds = poseEstimate.timestampSeconds;
    rv.latency = poseEstimate.latency;
    rv.avgTagDist = poseEstimate.avgTagDist;
    rv.fiducialIds = new int[poseEstimate.rawFiducials.length];
    for (int i = 0; i < rv.fiducialIds.length; ++i) {
      rv.fiducialIds[i] = poseEstimate.rawFiducials[i].id;
    }

    return rv;
  }

  public static final MegatagPoseEstimateStruct struct = new MegatagPoseEstimateStruct();
}
