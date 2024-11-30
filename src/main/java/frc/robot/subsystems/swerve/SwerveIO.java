package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {

  @AutoLog
  public static class SwerveIOInputs {
    public Pose2d Pose = new Pose2d();
    public ChassisSpeeds Speeds = new ChassisSpeeds();

    public SwerveModuleState[] ModuleStates = new SwerveModuleState[4];
    public SwerveModuleState[] ModuleTargets = new SwerveModuleState[4];
    public SwerveModulePosition[] ModulePositions = new SwerveModulePosition[4];

    public double OdometryPeriod = 0.0;
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;
  }

  default void updateInputs(SwerveIOInputs inputs) {}

  default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  default void resetPose(Pose2d pose) {}

  default void setControl(SwerveRequest request) {}

  default void seedFieldCentric() {}

  default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}
}
