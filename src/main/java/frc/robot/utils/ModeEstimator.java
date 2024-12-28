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

/**
 * Handles pose estimation for both real-time and replay modes of a swerve drive robot. This class
 * wraps WPILib's SwerveDrivePoseEstimator and provides additional functionality for handling vision
 * measurements and replaying recorded data.
 */
public class ModeEstimator {
  private final DriveIO io;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveModulePosition[] modulePositions;

  /**
   * Constructs a new ModeEstimator.
   *
   * @param io The DriveIO interface for accessing real hardware or simulation
   * @param modulePositions Initial positions of all swerve modules
   */
  public ModeEstimator(DriveIO io, SwerveModulePosition[] modulePositions) {
    this.io = io;
    this.modulePositions = modulePositions;
    this.kinematics = new SwerveDriveKinematics(Constants.SWERVE_MODULE_OFFSETS);
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.fromDegrees(0), modulePositions, new Pose2d());
  }

  /**
   * Resets the pose estimator to a specific pose. In replay mode, resets the internal pose
   * estimator. In real-time mode, delegates to the IO interface.
   *
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    if (Constants.currentMode == Mode.REPLAY) {
      poseEstimator.resetPose(pose);
    }
    io.resetPose(pose);
  }

  /**
   * Samples the pose at a specific timestamp.
   *
   * @param timestamp The timestamp to sample at (using WPILib timestamp)
   * @return Optional containing the pose at the given timestamp, or empty if not available
   */
  public Optional<Pose2d> samplePoseAt(double timestamp) {
    return Constants.currentMode == Mode.REPLAY
        ? poseEstimator.sampleAt(timestamp)
        : io.samplePoseAt(timestamp);
  }

  /**
   * Updates the pose estimator with recorded data during replay mode. This method processes arrays
   * of timestamps and corresponding robot state data.
   *
   * @param timestamp Array of timestamps
   * @param gyroYaw Array of gyro yaw measurements
   * @param drivePosition 2D array of drive positions for each module
   * @param steerPosition 2D array of steer positions for each module
   */
  public void updateWithTime(
      double[] timestamp,
      Rotation2d[] gyroYaw,
      double[][] drivePosition,
      Rotation2d[][] steerPosition) {
    if (Constants.currentMode == Mode.REPLAY) {
      for (int i = 0; i < timestamp.length; i++) {
        for (int moduleIndex = 0; moduleIndex < modulePositions.length; moduleIndex++) {
          modulePositions[moduleIndex].distanceMeters = drivePosition[moduleIndex][i];
          modulePositions[moduleIndex].angle = steerPosition[moduleIndex][i];
        }
        poseEstimator.updateWithTime(timestamp[i], gyroYaw[i], modulePositions);
      }
    }
  }

  /**
   * Gets the current estimated pose of the robot.
   *
   * @return The current estimated pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionRobotPoseMeters The measured robot pose from vision
   * @param timestampSeconds The timestamp of the measurement
   * @param visionMeasurementStdDevs Standard deviation matrix for the measurement
   */
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

  /**
   * Adds a vision measurement using the VisionMeasurement wrapper class.
   *
   * @param visionMeasurement The vision measurement to add
   */
  public void addVisionMeasurement(VisionMeasurement visionMeasurement) {
    this.addVisionMeasurement(
        visionMeasurement.poseEstimate().pose().toPose2d(),
        visionMeasurement.poseEstimate().timestampSeconds(),
        visionMeasurement.visionMeasurementStdDevs());
  }

  /**
   * Adds multiple vision measurements from a list.
   *
   * @param visionData List of vision measurements to add
   */
  public void addVisionData(List<VisionMeasurement> visionData) {
    visionData.forEach(this::addVisionMeasurement);
  }
}
