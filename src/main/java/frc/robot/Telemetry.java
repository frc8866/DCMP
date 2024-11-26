package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Telemetry {

  SwerveDriveState state;
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    this.state = state;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeedsMeasured() {
    return state.Speeds;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Setpoints")
  private ChassisSpeeds getChassisSpeedsSetpoints() {
    return kinematics.toChassisSpeeds(getStatesSetpoints());
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getStatesMeasured() {
    return state.ModuleStates;
  }

  @AutoLogOutput(key = "SwerveStates/Setpoints")
  private SwerveModuleState[] getStatesSetpoints() {
    return state.ModuleTargets;
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
