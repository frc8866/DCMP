package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/**
 * Drives the swerve drivetrain in a field-centric manner, maintaining a specified heading angle to
 * ensure the robot is facing the desired direction. Rotation to the target direction is profiled
 * using a trapezoid profile.
 *
 * <p>When users use this request, they specify the direction the robot should travel oriented
 * against the field, and the direction the robot should be facing.
 *
 * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
 * VelocityY is 0 m/s, and TargetDirection is 180 degrees. In this scenario, the robot would drive
 * northward at 5 m/s and turn clockwise to a target of 180 degrees.
 *
 * <p>This control request is especially useful for autonomous control, where the robot should be
 * facing a changing direction throughout the motion.
 */
public class SwerveSetpointGen implements NativeSwerveRequest {
  /**
   * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   */
  public double VelocityX = 0;
  /**
   * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   */
  public double VelocityY = 0;
  /**
   * The angular rate to rotate at, in radians per second. Angular rate is defined as
   * counterclockwise positive, so this determines how fast to turn counterclockwise.
   */
  public double RotationalRate = 0;

  /** The allowable deadband of the request, in m/s. */
  public double Deadband = 0;
  /** The rotational deadband of the request, in radians per second. */
  public double RotationalDeadband = 0;
  /**
   * The center of rotation the robot should rotate around. This is (0,0) by default, which will
   * rotate around the center of the robot.
   */
  public Translation2d CenterOfRotation = new Translation2d();

  /** The type of control request to use for the drive motor. */
  public SwerveModule.DriveRequestType DriveRequestType =
      SwerveModule.DriveRequestType.OpenLoopVoltage;
  /** The type of control request to use for the steer motor. */
  public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
  /**
   * Whether to desaturate wheel speeds before applying. For more information, see the documentation
   * of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
   */
  public boolean DesaturateWheelSpeeds = false;

  /** The perspective to use when determining which direction is forward. */
  public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

  private final FieldCentric m_swerveSetpoint = new FieldCentric();

  private SwerveSetpoint previousSetpoint;

  /**
   * Creates a new profiled SwerveSetpoint request with the given constraints.
   *
   * @param constraints Constraints for the trapezoid profile
   */
  public SwerveSetpointGen() {
    ChassisSpeeds currentSpeeds =
        new ChassisSpeeds(); // Method to get current robot-relative chassis speeds
    SwerveModuleState[] currentStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
        }; // Method to get the current swerve module states
    previousSetpoint =
        new SwerveSetpoint(
            currentSpeeds, currentStates, DriveFeedforwards.zeros(Constants.PP_CONFIG.numModules));
  }

  public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
    return StatusCode.OK;
  }

  public void applyNative(int id) {
    previousSetpoint =
        Constants.setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            new ChassisSpeeds(VelocityX, VelocityY, RotationalRate), // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
            );

    m_swerveSetpoint
        .withVelocityX(previousSetpoint.robotRelativeSpeeds().vxMetersPerSecond)
        .withVelocityY(previousSetpoint.robotRelativeSpeeds().vyMetersPerSecond)
        .withRotationalRate(previousSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond)
        .withDeadband(Deadband)
        .withRotationalDeadband(RotationalDeadband)
        .withCenterOfRotation(CenterOfRotation)
        .withDriveRequestType(DriveRequestType)
        .withSteerRequestType(SteerRequestType)
        .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
        .withForwardPerspective(ForwardPerspective)
        .applyNative(id);
  }

  /**
   * Modifies the VelocityX parameter and returns itself.
   *
   * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   *
   * @param newVelocityX Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withVelocityX(double newVelocityX) {
    this.VelocityX = newVelocityX;
    return this;
  }

  /**
   * Modifies the VelocityX parameter and returns itself.
   *
   * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
   * convention, so this determines how fast to travel forward.
   *
   * @param newVelocityX Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withVelocityX(LinearVelocity newVelocityX) {
    this.VelocityX = newVelocityX.in(MetersPerSecond);
    return this;
  }

  /**
   * Modifies the VelocityY parameter and returns itself.
   *
   * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   *
   * @param newVelocityY Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withVelocityY(double newVelocityY) {
    this.VelocityY = newVelocityY;
    return this;
  }

  /**
   * Modifies the VelocityY parameter and returns itself.
   *
   * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
   * convention, so this determines how fast to travel to the left.
   *
   * @param newVelocityY Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withVelocityY(LinearVelocity newVelocityY) {
    this.VelocityY = newVelocityY.in(MetersPerSecond);
    return this;
  }

  /**
   * Modifies the RotationalRate parameter and returns itself.
   *
   * <p>The angular rate to rotate at, in radians per second. Angular rate is defined as
   * counterclockwise positive, so this determines how fast to turn counterclockwise.
   *
   * @param newRotationalRate Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withRotationalRate(double newRotationalRate) {
    this.RotationalRate = newRotationalRate;
    return this;
  }

  /**
   * Modifies the RotationalRate parameter and returns itself.
   *
   * <p>The angular rate to rotate at, in radians per second. Angular rate is defined as
   * counterclockwise positive, so this determines how fast to turn counterclockwise.
   *
   * @param newRotationalRate Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withRotationalRate(AngularVelocity newRotationalRate) {
    this.RotationalRate = newRotationalRate.in(RadiansPerSecond);
    return this;
  }

  /**
   * Modifies the Deadband parameter and returns itself.
   *
   * <p>The allowable deadband of the request, in m/s.
   *
   * @param newDeadband Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withDeadband(double newDeadband) {
    this.Deadband = newDeadband;
    return this;
  }

  /**
   * Modifies the Deadband parameter and returns itself.
   *
   * <p>The allowable deadband of the request, in m/s.
   *
   * @param newDeadband Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withDeadband(LinearVelocity newDeadband) {
    this.Deadband = newDeadband.in(MetersPerSecond);
    return this;
  }

  /**
   * Modifies the RotationalDeadband parameter and returns itself.
   *
   * <p>The rotational deadband of the request, in radians per second.
   *
   * @param newRotationalDeadband Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withRotationalDeadband(double newRotationalDeadband) {
    this.RotationalDeadband = newRotationalDeadband;
    return this;
  }

  /**
   * Modifies the RotationalDeadband parameter and returns itself.
   *
   * <p>The rotational deadband of the request, in radians per second.
   *
   * @param newRotationalDeadband Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withRotationalDeadband(AngularVelocity newRotationalDeadband) {
    this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
    return this;
  }

  /**
   * Modifies the CenterOfRotation parameter and returns itself.
   *
   * <p>The center of rotation the robot should rotate around. This is (0,0) by default, which will
   * rotate around the center of the robot.
   *
   * @param newCenterOfRotation Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withCenterOfRotation(Translation2d newCenterOfRotation) {
    this.CenterOfRotation = newCenterOfRotation;
    return this;
  }

  /**
   * Modifies the DriveRequestType parameter and returns itself.
   *
   * <p>The type of control request to use for the drive motor.
   *
   * @param newDriveRequestType Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
    this.DriveRequestType = newDriveRequestType;
    return this;
  }

  /**
   * Modifies the SteerRequestType parameter and returns itself.
   *
   * <p>The type of control request to use for the drive motor.
   *
   * @param newSteerRequestType Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
    this.SteerRequestType = newSteerRequestType;
    return this;
  }

  /**
   * Modifies the DesaturateWheelSpeeds parameter and returns itself.
   *
   * <p>Whether to desaturate wheel speeds before applying. For more information, see the
   * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
   *
   * @param newDesaturateWheelSpeeds Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
    this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
    return this;
  }

  /**
   * Modifies the ForwardPerspective parameter and returns itself.
   *
   * <p>The perspective to use when determining which direction is forward.
   *
   * @param newForwardPerspective Parameter to modify
   * @return this object
   */
  public SwerveSetpointGen withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
    this.ForwardPerspective = newForwardPerspective;
    return this;
  }
}
