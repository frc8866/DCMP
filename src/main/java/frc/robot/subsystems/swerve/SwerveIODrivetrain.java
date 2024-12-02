package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SwerveIODrivetrain extends SwerveDrivetrain implements SwerveIO {

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public SwerveIODrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
    super(drivetrainConstants, modules);
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public SwerveIODrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(drivetrainConstants, OdometryUpdateFrequency, modules);
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation
   * @param visionStandardDeviation The standard deviation for vision calculation
   * @param modules Constants for each specific module
   */
  public SwerveIODrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
  }

  public Pose2d getPose() {
    return this.getState().Pose;
  }

  public Command applyRequest(
      Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
    return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    inputs.fromSwerveDriveState(super.getState());
  }

  @Override
  public void logModules(SwerveDriveState driveState) {
    final String[] moduleNames = {"Drive/FL/", "Drive/FR/", "Drive/BL/", "Drive/BR/"};
    for (int i = 0; i < driveState.ModuleStates.length; i++) {
      Logger.recordOutput(
          moduleNames[i] + "Drive Position",
          getModule(i).getDriveMotor().getPosition().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Drive Velocity",
          getModule(i).getDriveMotor().getVelocity().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Drive Applied Voltage",
          getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Drive Stator Current",
          getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Drive Supply Current",
          getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Turn Absolute Position",
          getModule(i).getCANcoder().getPosition().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Turn Position",
          getModule(i).getSteerMotor().getPosition().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Turn Velocity",
          getModule(i).getSteerMotor().getVelocity().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Turn Applied Voltage",
          getModule(i).getSteerMotor().getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Turn Stator Current",
          getModule(i).getSteerMotor().getStatorCurrent().getValueAsDouble());
      Logger.recordOutput(
          moduleNames[i] + "Turn Supply Current",
          getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble());
    }
  }
}
