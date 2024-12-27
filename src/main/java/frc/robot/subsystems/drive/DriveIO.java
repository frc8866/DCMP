// Copyright (c) 2024 FRC 5712
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
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
    public SwerveModulePosition[][] modulePositions = new SwerveModulePosition[][] {};
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
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  public default void setControl(SwerveRequest request) {}

  public default void seedFieldCentric() {}

  public default void resetPose(Pose2d pose) {}
}
