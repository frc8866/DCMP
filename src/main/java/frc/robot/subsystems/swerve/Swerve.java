// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionUtil.VisionMeasurement;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs;
  private final SwerveDriveState state;

  private final PIDConstants translationController = new PIDConstants(10, 0, 0);
  private final PIDConstants thetaController = new PIDConstants(7, 0, 0);

  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 69.78;
  private static final double ROBOT_MOI = 15.1702655465;
  private static final double WHEEL_COF = 1.9;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          Math.abs(TunerConstants.FrontLeft.LocationY - TunerConstants.FrontRight.LocationY),
          Math.abs(TunerConstants.FrontLeft.LocationX - TunerConstants.BackLeft.LocationX));

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              Seconds.of(6),
              // Log state with SignalLogger class
              sysIdState -> SignalLogger.writeString("SysIdTranslation_State", sysIdState.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              sysIdState -> SignalLogger.writeString("SysIdSteer_State", sysIdState.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              sysIdState -> SignalLogger.writeString("SysIdRotation_State", sysIdState.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private final SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  private void configureAutoBuilder() {
    AutoBuilder.configure(
        () -> this.state.Pose, // Supplier of current robot pose
        this::resetPose, // Consumer for seeding pose against auto
        () -> this.state.Speeds, // Supplier of current robot speeds
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        (speeds, feedforwards) ->
            setControl(
                m_pathApplyRobotSpeeds
                    .withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        new PPHolonomicDriveController(
            // PID constants for translation
            translationController,
            // PID constants for rotation
            thetaController),
        PP_CONFIG,
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this // Subsystem for requirements
        );
  }

  private boolean hasAppliedOperatorPerspective;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BLUE_PERSPECTIVE = Rotation2d.fromDegrees(0);

  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RED_PERSPECTIVE = Rotation2d.fromDegrees(180);

  public Swerve(SwerveIO io) {
    this.io = io;
    state = new SwerveDriveState();
    inputs = new SwerveIOInputsAutoLogged();
    configureAutoBuilder();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    state.FailedDaqs = inputs.failedDaqs;
    state.ModuleStates = inputs.moduleStates;
    state.ModuleTargets = inputs.moduleTargetStates;
    state.OdometryPeriod = inputs.odometryPeriodSeconds;
    state.Pose = inputs.pose;
    state.SuccessfulDaqs = inputs.successfulDaqs;
    state.Speeds = inputs.speeds;

    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                io.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? RED_PERSPECTIVE : BLUE_PERSPECTIVE);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> io.setControl(requestSupplier.get()));
  }

  public Command brake() {
    return applyRequest(() -> brakeRequest);
  }

  public SwerveDriveState getState() {
    return state;
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

  public Command seedFieldCentric() {
    return runOnce(io::seedFieldCentric);
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    io.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    io.addVisionMeasurement(visionMeasurement, timestampSeconds, visionMeasurementStdDevs);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(VisionMeasurement visionMeasurement) {
    addVisionMeasurement(
        visionMeasurement.poseEstimate().pose().toPose2d(),
        Utils.fpgaToCurrentTime(visionMeasurement.poseEstimate().timestampSeconds()),
        visionMeasurement.visionMeasurementStdDevs());
  }

  public void addVisionData(List<VisionMeasurement> visionData) {
    visionData.forEach(this::addVisionMeasurement);
  }

  public VisionParameters getVisionParameters() {
    return new VisionParameters(state.Pose, state.Speeds.omegaRadiansPerSecond);
  }

  public record VisionParameters(Pose2d robotPose, double yawVelocityRadPerSec) {}
}
