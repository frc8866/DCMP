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

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Arm subsystem controls a dual-motor arm mechanism for game piece manipulation. It supports
 * multiple positions for different game actions and provides both open-loop and closed-loop control
 * options.
 */
public class Arm extends SubsystemBase {
  // Hardware interface and inputs
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs;

  // Current arm position mode
  private ArmPosition currentMode = ArmPosition.INTAKE;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Arm leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Arm follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Arm encoder isn't connected", AlertType.kError);

  /**
   * Creates a new Arm subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the arm
   */
  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  /**
   * Runs the arm in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  private void setPosition(Angle position) {
    io.setPosition(position);
  }

  /** Stops the arm motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current position of the arm.
   *
   * @return The current angular position
   */
  @AutoLogOutput
  public Angle getPosition() {
    return inputs.encoderPosition;
  }

  /** Enumeration of available arm positions with their corresponding target angles. */
  private enum ArmPosition {
    STOP(Degrees.of(0)), // Stop the arm
    INTAKE(Degrees.of(0)), // Arm tucked in
    L1(Degrees.of(90)), // Position for scoring in L1
    L2(Degrees.of(135)), // Position for scoring in L2
    L3(Degrees.of(135)), // Position for scoring in L3
    L4(Degrees.of(180)); // Position for scoring in L4

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ArmPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ArmPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2)); // 2 degree default tolerance
    }
  }

  /**
   * Gets the current arm position mode.
   *
   * @return The current ArmPosition
   */
  public ArmPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new arm position and schedules the corresponding command.
   *
   * @param position The desired ArmPosition
   */
  private void setArmPosition(ArmPosition position) {
    currentCommand.cancel();
    currentMode = position;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ArmPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop Arm"),
              ArmPosition.INTAKE,
              createPositionCommand(ArmPosition.INTAKE),
              ArmPosition.L1,
              createPositionCommand(ArmPosition.L1),
              ArmPosition.L2,
              createPositionCommand(ArmPosition.L2),
              ArmPosition.L3,
              createPositionCommand(ArmPosition.L3),
              ArmPosition.L4,
              createPositionCommand(ArmPosition.L4)),
          this::getMode);

  /**
   * Creates a command for a specific arm position that moves the arm and checks the target
   * position.
   *
   * @param position The arm position to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ArmPosition position) {
    return Commands.runOnce(() -> setPosition(position.targetAngle))
        .withName("Move to " + position.toString());
  }

  /**
   * Checks if the arm is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == ArmPosition.STOP) return true;
    return getPosition().isNear(currentMode.targetAngle, currentMode.angleTolerance);
  }

  /**
   * Logs target angle for given mode.
   *
   * @return The target angle for the current mode
   */
  @AutoLogOutput
  private Angle targetAngle() {
    return currentMode.targetAngle;
  }

  /**
   * Creates a command to set the arm to a specific position.
   *
   * @param position The desired arm position
   * @return Command to set the position
   */
  private Command setPositionCommand(ArmPosition position) {
    return Commands.runOnce(() -> setArmPosition(position))
        .withName("SetArmPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to move the arm to L1 scoring position
   */
  public final Command L1() {
    return setPositionCommand(ArmPosition.L1);
  }

  /**
   * @return Command to move the arm to L2 scoring position
   */
  public final Command L2() {
    return setPositionCommand(ArmPosition.L2);
  }

  /**
   * @return Command to move the arm to L3 position
   */
  public final Command L3() {
    return setPositionCommand(ArmPosition.L3);
  }

  /**
   * @return Command to move the arm to L4 position
   */
  public final Command L4() {
    return setPositionCommand(ArmPosition.L4);
  }

  /**
   * @return Command to intake the arm
   */
  public final Command intake() {
    return setPositionCommand(ArmPosition.INTAKE);
  }

  /**
   * @return Command to stop the arm
   */
  public final Command stopCommand() {
    return setPositionCommand(ArmPosition.STOP);
  }
}
