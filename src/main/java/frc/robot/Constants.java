// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveIOCTRE;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final DriveConstants DRIVE_CONSTANTS =
      DriveConstants.from(
          TunerConstants.kSpeedAt12Volts,
          RotationsPerSecond.of(0.75),
          TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0,
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static record DriveConstants(
      LinearVelocity kSpeedAt12Volts,
      AngularVelocity kMaxTurnRate,
      double frequency,
      double WheelRadius,
      double DriveMotorGearRatio,
      double SlipCurrent,
      SwerveDrivetrainConstants drivetrainConstants,
      DriveIOCTRE drivetrain) {
    public static DriveConstants from(
        LinearVelocity kSpeedAt12Volts,
        AngularVelocity kMaxTurnRate,
        double frequency,
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants... modules) {
      return new DriveConstants(
          kSpeedAt12Volts,
          kMaxTurnRate,
          frequency,
          modules[0].WheelRadius,
          modules[0].DriveMotorGearRatio,
          modules[0].SlipCurrent,
          drivetrainConstants,
          new DriveIOCTRE(drivetrainConstants, modules));
    }
  }
}
