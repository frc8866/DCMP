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

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs;

  private final Alert leaderMotorAlert =
      new Alert("Flywheel leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Flywheel follower motor isn't connected", AlertType.kError);

  private ShotMode currentMode = ShotMode.NONE;

  private final SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              state -> Logger.recordOutput("state", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                this.runVoltage(output);
                Logger.recordOutput("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.inputs = new FlywheelIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
  }

  public ShotMode getMode() {
    return currentMode;
  }

  public void setMode(ShotMode mode) {
    getCommand.cancel();
    currentMode = mode;
    getCommand.schedule();
  }

  public enum ShotMode {
    NONE,
    AMP,
    SPEAKER,
    FEED
  }

  private final Command getCommand =
      new SelectCommand<>(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(ShotMode.NONE, new PrintCommand("No command was selected!")),
              Map.entry(ShotMode.AMP, new PrintCommand("Command one was selected!")),
              Map.entry(ShotMode.SPEAKER, new PrintCommand("Command two was selected!")),
              Map.entry(ShotMode.FEED, new PrintCommand("Command three was selected!"))),
          this::getMode);

  /** Run open loop at the specified voltage. */
  public void runVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  public void runVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }
}
