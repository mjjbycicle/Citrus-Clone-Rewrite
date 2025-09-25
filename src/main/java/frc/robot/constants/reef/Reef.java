package frc.robot.constants.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;

public class Reef {
  public static final FieldType fieldType = FieldType.WELDED;
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  private static final Pose2d[] centerFaces = new Pose2d[6];

  static {
    // Initialize faces
    var aprilTagLayout = AprilTagLayoutType.OFFICIAL.getLayout();
    centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
    centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
    centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
    centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
    centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
    centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();
  }

  public Pose2d getReefFacePose(int faceIndex, DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      return AllianceFlipUtil.apply(centerFaces[faceIndex]);
    }
    return centerFaces[faceIndex];
  }

  public Pose2d getClosestReefFacePose(Pose2d robotPose, DriverStation.Alliance alliance) {
    List<Pose2d> reefFacePoses = new ArrayList<>();
    for (int i = 0; i < 6; i++) {
      reefFacePoses.add(getReefFacePose(i, alliance));
    }
    return robotPose.nearest(reefFacePoses);
  }
}
