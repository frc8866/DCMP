// Copyright (c) 2024 FRC 5712
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class DriveIOCTRE extends SwerveDrivetrain implements DriveIO {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier;
  private double m_lastSimTime;

  // queues for odometry updates from CTRE's thread
  private final Lock odometryLock = new ReentrantLock();
  Queue<Rotation2d> gyroYawQueue = new ArrayBlockingQueue<>(20);
  Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);
  List<Queue<Double>> drivePositionQueues = new ArrayList<>();
  List<Queue<Rotation2d>> steerPositionQueues = new ArrayList<>();

  public DriveIOCTRE(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    for (int i = 0; i < 4; i++) {
      this.drivePositionQueues.add(new ArrayBlockingQueue<>(20));
      this.steerPositionQueues.add(new ArrayBlockingQueue<>(20));
    }

    this.registerTelemetry(this::updateTelemetry);
    if (Constants.currentMode == Mode.SIM) {
      startSimThread();
    }
  }

  public DriveIOCTRE(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    for (int i = 0; i < 4; i++) {
      this.drivePositionQueues.add(new ArrayBlockingQueue<>(20));
      this.steerPositionQueues.add(new ArrayBlockingQueue<>(20));
    }

    this.registerTelemetry(this::updateTelemetry);
    if (Constants.currentMode == Mode.SIM) {
      startSimThread();
    }
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    SwerveDriveState state = super.getState();
    inputs.moduleStates = state.ModuleStates;
    inputs.moduleTargets = state.ModuleTargets;
    inputs.pose = state.Pose;
    inputs.speeds = state.Speeds;
    inputs.odometryPeriod = state.OdometryPeriod;
    inputs.successfulDaqs = state.SuccessfulDaqs;
    inputs.failedDaqs = state.FailedDaqs;

    inputs.gyroRate = super.getPigeon2().getAngularVelocityZWorld().getValue();
    inputs.operatorForwardDirection = super.getOperatorForwardDirection();
    inputs.odometryIsValid = super.isOdometryValid();

    this.odometryLock.lock();

    inputs.timestamp = this.timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    this.timestampQueue.clear();

    inputs.gyroYaw = this.gyroYawQueue.stream().toArray(Rotation2d[]::new);
    this.gyroYawQueue.clear();

    for (int i = 0; i < this.getModules().length; i++) {
      inputs.drivePositions[i] =
          this.drivePositionQueues.get(i).stream().mapToDouble(Double::valueOf).toArray();
      inputs.steerPositions[i] =
          this.steerPositionQueues.get(i).stream().toArray(Rotation2d[]::new);

      this.drivePositionQueues.get(i).clear();
      this.steerPositionQueues.get(i).clear();
    }

    this.odometryLock.unlock();
  }

  private void updateTelemetry(SwerveDriveState state) {
    this.odometryLock.lock();

    for (int i = 0; i < state.ModuleStates.length; i++) {
      this.drivePositionQueues.get(i).offer(state.ModulePositions[i].distanceMeters);
      this.steerPositionQueues.get(i).offer(state.ModuleStates[i].angle);
    }

    this.gyroYawQueue.offer(state.RawHeading);

    this.timestampQueue.offer(
        Timer.getFPGATimestamp() - (Utils.getCurrentTimeSeconds() - state.Timestamp));

    this.odometryLock.unlock();
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }
}
