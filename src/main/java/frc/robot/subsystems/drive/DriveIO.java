// Copyright (c) 2024 FRC 5712
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public SwerveModuleState[] moduleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
    public SwerveModuleState[] moduleTargets =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
    public SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    public Pose2d pose = Pose2d.kZero;
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public double odometryPeriod = 0.0;
    public int successfulDaqs = 0;
    public int failedDaqs = 0;
    public double timestamp[] = new double[] {};

    public Rotation2d[] gyroYaw = new Rotation2d[] {};
    public AngularVelocity gyroRate = RotationsPerSecond.of(0.0);

    public Rotation2d operatorForwardDirection = new Rotation2d();
    public boolean odometryIsValid = false;

    public double[][] drivePositions =
        new double[][] {new double[] {}, new double[] {}, new double[] {}, new double[] {}};
    public Rotation2d[][] steerPositions =
        new Rotation2d[][] {
          new Rotation2d[] {}, new Rotation2d[] {}, new Rotation2d[] {}, new Rotation2d[] {}
        };
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  public default void setControl(SwerveRequest request) {}

  public default void resetPose(Pose2d pose) {}

  public default Optional<Pose2d> samplePoseAt(double timestamp) {
    return Optional.empty();
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * PoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * PoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link #updateWithTime}, then you must use a
   *     timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same
   *     epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}). This means that you
   *     should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source in
   *     this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}
}
