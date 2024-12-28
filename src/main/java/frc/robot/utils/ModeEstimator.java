package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.vision.VisionUtil.VisionMeasurement;
import java.util.List;
import java.util.Optional;

public class ModeEstimator {
  private DriveIO io;
  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.SWERVE_MODULE_OFFSETS);
  private SwerveModulePosition[] initPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, initPositions, new Pose2d());

  public ModeEstimator(DriveIO io) {
    this.io = io;
  }

  public void resetPose(Pose2d pose) {
    if (Constants.currentMode == Mode.REPLAY) {
      poseEstimator.resetPose(pose);
    } else {
      io.resetPose(pose);
    }
  }

  /**
   * Return the pose at a given timestamp. If the buffer is empty return current pose.
   *
   * @param timestampSeconds The pose's timestamp. This must use WPILib timestamp.
   * @return The pose at the given timestamp (or current pose if the buffer is empty).
   */
  public Optional<Pose2d> samplePoseAt(double timestamp) {
    if (Constants.currentMode == Mode.REPLAY) {
      return poseEstimator.sampleAt(timestamp);
    } else {
      return io.samplePoseAt(timestamp);
    }
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (Constants.currentMode == Mode.REPLAY) {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    } else {
      io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
  }

  public void updateWithTime(
      double[] timestamp, Rotation2d[] gyroYaw, SwerveModulePosition[][] modulePositions) {
    if (Constants.currentMode == Mode.REPLAY) {
      for (int i = 0; i < timestamp.length; i++) {
        poseEstimator.updateWithTime(timestamp[i], gyroYaw[i], modulePositions[i]);
      }
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(VisionMeasurement visionMeasurement) {
    poseEstimator.addVisionMeasurement(
        visionMeasurement.poseEstimate().pose().toPose2d(),
        visionMeasurement.poseEstimate().timestampSeconds(),
        visionMeasurement.visionMeasurementStdDevs());
  }

  public void addVisionData(List<VisionMeasurement> visionData) {
    visionData.forEach(this::addVisionMeasurement);
  }
}
