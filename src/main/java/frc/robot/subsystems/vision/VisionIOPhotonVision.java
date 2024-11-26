// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.VisionParameters;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final Supplier<VisionParameters> visionParams;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(
      String cameraName, Transform3d robotToCamera, Supplier<VisionParameters> visionParams) {
    camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.visionParams = visionParams;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    PoseObservation poseObservations = getEstimatedGlobalPose();
    inputs.poseEstimateMT1 = poseObservations.poseEstimate();
    inputs.rawFiducialsMT1 = poseObservations.rawFiducials();
  }

  public PoseObservation getEstimatedGlobalPose() {
    List<PhotonPipelineResult> allResults = camera.getAllUnreadResults();
    if (allResults.isEmpty() || allResults.size() - 1 < 0) {
      return new PoseObservation();
    }

    PhotonPipelineResult result = allResults.get(allResults.size() - 1);
    if (result.hasTargets() && result.getMultiTagResult().isPresent()) {
      var multitagResult = result.multitagResult.get();
      Transform3d fieldToCamera = multitagResult.estimatedPose.best;
      Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

      // Calculate average tag distance
      double totalTagDistance = 0.0;
      double totalTagArea = 0.0;
      List<RawFiducial> rawFiducialsList = new ArrayList<>();
      for (var target : result.targets) {
        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        totalTagArea += target.area;
        rawFiducialsList.add(
            new RawFiducial(
                target.getFiducialId(),
                0,
                0,
                target.area,
                target
                    .bestCameraToTarget
                    .getTranslation()
                    .minus(robotToCamera.getTranslation())
                    .getNorm(),
                target.bestCameraToTarget.getTranslation().getNorm(),
                target.poseAmbiguity));
      }
      int tagCount = result.targets.size();
      double avgTagDistance = tagCount > 0 ? totalTagDistance / tagCount : 0.0;
      double avgTagArea = tagCount > 0 ? totalTagArea / tagCount : 0.0;
      double ambiguity = tagCount > 0 ? rawFiducialsList.get(0).ambiguity() : 0.0;

      RawFiducial[] rawFiducials = rawFiducialsList.toArray(new RawFiducial[0]);
      VisionParameters currentParams = this.visionParams.get();
      PoseEstimate poseEstimate =
          new PoseEstimate(
              robotPose,
              result.getTimestampSeconds(),
              0.0,
              tagCount,
              0.0,
              avgTagDistance,
              avgTagArea,
              ambiguity,
              currentParams.yawVelocityRadPerSec(),
              currentParams.robotPose(),
              false);

      return new PoseObservation(poseEstimate, rawFiducials);
    }
    return new PoseObservation();
  }
}
