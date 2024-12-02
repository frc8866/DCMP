package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {

  @AutoLog
  public static class SwerveIOInputs extends SwerveDriveState {
    SwerveIOInputs() {
      this.Pose = Pose2d.kZero;
    }

    public void fromSwerveDriveState(SwerveDriveState state) {
      this.Pose = state.Pose;
      this.Speeds = state.Speeds;

      this.ModuleStates = state.ModuleStates;
      this.ModuleTargets = state.ModuleTargets;
      this.ModulePositions = state.ModulePositions;

      this.OdometryPeriod = state.OdometryPeriod;
      this.SuccessfulDaqs = state.SuccessfulDaqs;
      this.FailedDaqs = state.FailedDaqs;
    }
  }

  void updateInputs(SwerveIOInputs inputs);

  void logModules(SwerveDriveState driveState);

  void resetPose(Pose2d pose);

  Pose2d getPose();

  Translation2d[] getModuleLocations();

  void setControl(SwerveRequest request);

  Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

  void setOperatorPerspectiveForward(Rotation2d forward);

  void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
}
