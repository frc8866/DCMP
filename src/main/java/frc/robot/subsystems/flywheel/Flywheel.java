// Copyright FRC 5712
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.ControlledMechanism;
import frc.robot.utils.ControlledMechanism.TargetProvider;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * The Flywheel subsystem controls a dual-motor flywheel mechanism used for shooting game pieces. It
 * supports multiple shooting modes with different target speeds and provides closed-loop control.
 */
public class Flywheel extends ControlledMechanism<AngularVelocity, Flywheel.FlywheelPosition> {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs;
  private final Alert leaderMotorAlert;
  private final Alert followerMotorAlert;

  /** Enumeration of available flywheel positions with their corresponding target speeds. */
  public enum FlywheelPosition implements TargetProvider<AngularVelocity> {
    STOP(RotationsPerSecond.of(0)),
    L1(RotationsPerSecond.of(5)),
    PROCESSOR(RotationsPerSecond.of(10)),
    NET(RotationsPerSecond.of(20));

    private final AngularVelocity targetSpeed;
    private final AngularVelocity speedTolerance;

    FlywheelPosition(AngularVelocity targetSpeed, AngularVelocity speedTolerance) {
      this.targetSpeed = targetSpeed;
      this.speedTolerance = speedTolerance;
    }

    FlywheelPosition(AngularVelocity targetSpeed) {
      this(targetSpeed, RotationsPerSecond.of(1)); // 1 RPS default tolerance
    }

    @Override
    public AngularVelocity getTarget() {
      return targetSpeed;
    }

    @Override
    public AngularVelocity getTolerance() {
      return speedTolerance;
    }
  }

  public Flywheel(FlywheelIO io) {
    super(FlywheelPosition.STOP); // Set initial position
    this.io = io;
    this.inputs = new FlywheelIOInputsAutoLogged();

    this.leaderMotorAlert = new Alert("Flywheel leader motor isn't connected", AlertType.kError);
    this.followerMotorAlert =
        new Alert("Flywheel follower motor isn't connected", AlertType.kError);
  }

  @Override
  protected void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  @Override
  protected void updateAlerts() {
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
  }

  @Override
  protected void setTarget(AngularVelocity target) {
    io.setVelocity(target);
  }

  @Override
  protected void stop() {
    io.stop();
  }

  @Override
  public AngularVelocity getCurrentValue() {
    return inputs.velocity;
  }

  @Override
  protected Map<FlywheelPosition, Command> createCommandMap() {
    return Map.of(
        FlywheelPosition.STOP, Commands.runOnce(this::stop).withName("Stop Flywheel"),
        FlywheelPosition.L1, createStateCommand(FlywheelPosition.L1),
        FlywheelPosition.PROCESSOR, createStateCommand(FlywheelPosition.PROCESSOR),
        FlywheelPosition.NET, createStateCommand(FlywheelPosition.NET));
  }

  @Override
  protected boolean isStopState(FlywheelPosition mode) {
    return mode == FlywheelPosition.STOP;
  }

  @Override
  protected boolean isNear(
      AngularVelocity current, AngularVelocity target, AngularVelocity tolerance) {
    return current.isNear(target, tolerance);
  }

  /**
   * @return Command to set the flywheel to L1 position
   */
  public final Command L1() {
    return createStateCommand(FlywheelPosition.L1);
  }

  /**
   * @return Command to set the flywheel to PROCESSOR position
   */
  public final Command processor() {
    return createStateCommand(FlywheelPosition.PROCESSOR);
  }

  /**
   * @return Command to set the flywheel to NET position
   */
  public final Command net() {
    return createStateCommand(FlywheelPosition.NET);
  }

  /**
   * @return Command to stop the flywheel
   */
  public final Command stopCommand() {
    return createStateCommand(FlywheelPosition.STOP);
  }
}
