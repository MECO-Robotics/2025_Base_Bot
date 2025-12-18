package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionIOQuestNavNew.QuestNavData;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

public class VisionIOQuestNavRelative implements VisionIO {

  private QuestNav questNav;

  // Local heading helper variables
  // private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();

  private final Transform3d robotToCamera;

  private Transform3d offset = new Transform3d();
  private boolean hasAllianceReset = false;

  private Pose3d lastpose;

  public VisionIOQuestNavRelative(Transform3d robotToCamera) {
    // Initialize the camera to robot transform
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if (!hasAllianceReset) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          zeroPosition();
          offset =
              new Transform3d(
                  new Pose3d(
                      new Translation3d(5.643, 4, 0).plus(robotToCamera.getTranslation()),
                      new Rotation3d(0, 0, Math.toRadians(0)).plus(robotToCamera.getRotation())),
                  new Pose3d());
          hasAllianceReset = true;
        } else {
          zeroPosition();
          offset =
              new Transform3d(
                  new Pose3d(
                      new Translation3d(11.893, 4, 0).plus(robotToCamera.getTranslation()),
                      new Rotation3d(0, 0, Math.toRadians(180)).plus(robotToCamera.getRotation())),
                  new Pose3d(getPose()));
          hasAllianceReset = true;
        }
      }
    }

    QuestNavData[] questNavData = getQuestNavData();

    inputs.connected = connected();
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d(), 0);
    inputs.poseObservations = new PoseObservation[questNavData.length];

    for (int i = 0; i < questNavData.length; i++) {
      inputs.poseObservations[i] =
          new PoseObservation(
              questNavData[i].timestamp(),
              new Pose3d(
                  questNavData[i].pose().getTranslation().plus(offset.getTranslation()),
                  questNavData[i].pose().getRotation().plus(offset.getRotation())),
              0.0,
              -1,
              0.0,
              PoseObservationType.QUESTNAV);
    }
    inputs.tagIds = new int[0];

    Logger.recordOutput("QuestNav/battery", getBatteryPercent());

    Logger.recordOutput("QuestNav/offset", offset);

    Logger.recordOutput("QuestNav/RawPose", getPose());

    Logger.recordOutput("QuestNav/AllianceReset", hasAllianceReset);
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the
  // quest logo)
  public void zeroPosition() {
    resetPosition = getPose();
  }

  private QuestNavData[] getQuestNavData() {

    PoseFrame[] newFrame = questNav.getAllUnreadPoseFrames();
    double battery = getBatteryPercent();
    int length = newFrame.length;
    QuestNavData[] data = new QuestNavData[length];

    for (int i = 0; i < length; i++) {
      data[i] =
          new QuestNavData(
              newFrame[i].questPose3d().plus(robotToCamera.inverse()),
              battery,
              newFrame[i].dataTimestamp(),
              getQuestTranslation(newFrame[i].questPose3d()),
              getQuestRotation(newFrame[i].questPose3d().getRotation()));

      lastpose = newFrame[i].questPose3d();
    }

    return data;
  }

  private boolean connected() {
    return questNav.isConnected();
  }

  private double getBatteryPercent() {
    return questNav.getBatteryPercent().orElse(0);
  }

  private float[] getQuestTranslation(Pose3d pose) {
    // TODO: Check if translation floats are correct.
    float xPosition = (float) pose.getX();
    float yPosition = (float) pose.getY();
    float zPosition = (float) pose.getZ();

    return new float[] {-yPosition, zPosition, xPosition};
  }

  private float[] getQuestRotation(Rotation3d angle) {

    // TODO: Check if yaw and roll are good when inversed
    float yaw = (float) -Units.radiansToDegrees(angle.getZ());
    float pitch = (float) Units.radiansToDegrees(angle.getY());
    float roll = (float) -Units.radiansToDegrees(angle.getX());

    return new float[] {pitch, yaw, roll};
  }

  // Gets the Quest's measured position.
  public Pose2d getPose() {
    return new Pose2d(
        lastpose.toPose2d().minus(resetPosition).getTranslation(),
        lastpose.toPose2d().getRotation());
  }
}
