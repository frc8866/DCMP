package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionUtil.VisionData;
import frc.robot.subsystems.vision.VisionUtil.VisionMeasurement;
import frc.robot.subsystems.vision.VisionUtil.VisionMode;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;
  //  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private static final VisionMode mode = VisionMode.MA;

  private static final String VISION_PATH = "Vision/Camera";

  public Vision(VisionConsumer consumer, VisionIO... io) {
    System.out.println("[Init] Creating Vision");
    this.consumer = consumer;
    this.io = io;
    inputs = new VisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
    }
    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);
      // Logger.processInputs(VISION_PATH + Integer.toString(i), inputs[i]);
    }
    VisionData visionData = processAllCameras();
    logSummary(visionData);
    consumer.accept(sortMeasurements(visionData.measurements()));
  }

  private VisionData processAllCameras() {
    VisionData combinedData = VisionData.empty();
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      VisionData cameraData = processCamera(cameraIndex, inputs[cameraIndex]);
      combinedData = combinedData.merge(cameraData);
    }
    return combinedData;
  }

  private VisionData processCamera(int cameraIndex, VisionIOInputs input) {
    PoseObservation[] poseObservations = {
      new PoseObservation(input.poseEstimateMT1, input.rawFiducialsMT1),
      new PoseObservation(input.poseEstimateMT2, input.rawFiducialsMT2)
    };

    return Arrays.stream(poseObservations)
        .filter(PoseObservation::isValid)
        .map(observation -> processObservation(cameraIndex, observation))
        .reduce(VisionData.empty(), VisionData::merge);
  }

  private VisionData processObservation(int cameraIndex, PoseObservation observation) {
    // Create lists for measurements and poses
    List<VisionMeasurement> measurements = new ArrayList<>();
    List<Pose3d> tagPoses = new ArrayList<>();
    List<Pose3d> acceptedTagPoses = new ArrayList<>();
    List<Pose3d> rejectedTagPoses = new ArrayList<>();
    List<Pose3d> robotPoses = new ArrayList<>();
    List<Pose3d> acceptedPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    var robotPose = observation.poseEstimate().pose();
    // Add robot pose
    robotPoses.add(robotPose);

    // Process tag poses. If tag exist in FieldConstants.aprilTags, add to tagPoses
    boolean acceptedVisionMeasurement = mode.acceptVisionMeasurement(observation);
    for (var tag : observation.rawFiducials()) {
      FieldConstants.aprilTags
          .getTagPose(tag.id())
          .ifPresent(
              pose -> {
                tagPoses.add(pose);
                if (acceptedVisionMeasurement) {
                  acceptedTagPoses.add(pose);
                } else {
                  rejectedTagPoses.add(pose);
                }
              });
    }

    // Handle acceptance/rejection
    if (acceptedVisionMeasurement) {
      measurements.add(mode.getVisionMeasurement(observation.poseEstimate()));
      acceptedPoses.add(robotPose);
    } else {
      rejectedPoses.add(robotPose);
    }

    // Create VisionData object to return for specific camera
    var data =
        new VisionData(
            measurements,
            tagPoses,
            acceptedTagPoses,
            rejectedTagPoses,
            robotPoses,
            acceptedPoses,
            rejectedPoses);

    String mtType = observation.poseEstimate().isMegaTag2() ? "/MegaTag2" : "/MegaTag1";
    logCameraData(cameraIndex, mtType, data);
    return data;
  }

  private void logCameraData(int cameraIndex, String mtType, VisionData data) {
    logPoses(VISION_PATH + cameraIndex + mtType, data);
  }

  private void logSummary(VisionData data) {
    logPoses("Vision/Summary", data);
  }

  private void logPoses(String basePath, VisionData data) {
    Logger.recordOutput(basePath + "/TagPoses", toPose3dArray(data.tagPoses()));
    Logger.recordOutput(basePath + "/TagPosesAccepted", toPose3dArray(data.acceptedTagPoses()));
    Logger.recordOutput(basePath + "/TagPosesRejected", toPose3dArray(data.rejectedTagPoses()));
    Logger.recordOutput(basePath + "/RobotPoses", toPose3dArray(data.robotPoses()));
    Logger.recordOutput(basePath + "/RobotPosesAccepted", toPose3dArray(data.acceptedPoses()));
    Logger.recordOutput(basePath + "/RobotPosesRejected", toPose3dArray(data.rejectedPoses()));
  }

  private Pose3d[] toPose3dArray(List<Pose3d> poses) {
    return poses.toArray(new Pose3d[poses.size()]);
  }

  private List<VisionMeasurement> sortMeasurements(List<VisionMeasurement> measurements) {
    return measurements.stream()
        .sorted(
            (vm1, vm2) ->
                Double.compare(
                    vm1.poseEstimate().timestampSeconds(), vm2.poseEstimate().timestampSeconds()))
        .toList();
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(List<VisionMeasurement> visionMeasurements);
  }
}
