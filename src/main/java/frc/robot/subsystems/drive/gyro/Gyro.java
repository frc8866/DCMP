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

package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Gyro {
  private final GyroIO io;
  private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  private final Alert gyroDisconnectedAlert;

  public Gyro(GyroIO io) {
    this.io = io;
    gyroDisconnectedAlert = new Alert("Gyro is Disconnected.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    // Update alerts
    gyroDisconnectedAlert.set(!inputs.connected);
  }

  public Angle getYaw() {
    return inputs.yawPosition;
  }

  public AngularVelocity getYawVelocity() {
    return inputs.yawVelocity;
  }

  public boolean isConnected() {
    return inputs.connected;
  }
}
