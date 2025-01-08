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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Arm leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Arm follower motor isn't connected", AlertType.kError);
  private final Alert encoderAlert = new Alert("Arm encoder isn't connected", AlertType.kError);

  // System identification routine configuration
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
                Logger.recordOutput("Arm_Position", output.in(Volts));
              },
              null,
              this));

  // Current arm position mode
  private ArmPosition currentPosition = ArmPosition.STOWED;

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
   * Runs the arm in open-loop mode at the specified voltage.
   *
   * @param volts The voltage to apply to the motors
   */
  public void runVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  /**
   * Runs the arm in closed-loop position mode to the specified angle.
   *
   * @param position The target angle position
   */
  public void setPosition(Angle position) {
    io.setPosition(position);
  }

  /** Stops the arm motors. */
  public void stop() {
    io.stop();
  }

  /**
   * Returns a command to run a quasistatic system identification test.
   *
   * @param direction The direction to run the test
   * @return The command to run the test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /**
   * Returns a command to run a dynamic system identification test.
   *
   * @param direction The direction to run the test
   * @return The command to run the test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
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
  public enum ArmPosition {
    STOP(Degrees.of(0)), // Stop the arm
    STOWED(Degrees.of(0)), // Arm tucked in
    INTAKE(Degrees.of(30)), // Position for intaking from ground
    AMP(Degrees.of(90)), // Position for scoring in amp
    SPEAKER(Degrees.of(60)); // Position for scoring in speaker

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
  public ArmPosition getArmPosition() {
    return currentPosition;
  }

  /**
   * Sets a new arm position and schedules the corresponding command.
   *
   * @param position The desired ArmPosition
   */
  public void setArmPosition(ArmPosition position) {
    currentCommand.cancel();
    currentPosition = position;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ArmPosition.STOP,
              Commands.runOnce(this::stop)
                  .alongWith(Commands.run(() -> checkAtTarget(ArmPosition.STOP)))
                  .withName("Stop Arm"),
              ArmPosition.STOWED,
              createPositionCommand(ArmPosition.STOWED),
              ArmPosition.INTAKE,
              createPositionCommand(ArmPosition.INTAKE),
              ArmPosition.AMP,
              createPositionCommand(ArmPosition.AMP),
              ArmPosition.SPEAKER,
              createPositionCommand(ArmPosition.SPEAKER)),
          this::getArmPosition);

  /**
   * Creates a command for a specific arm position that moves the arm and checks the target
   * position.
   *
   * @param position The arm position to create a command for
   * @return A command that implements the arm movement
   */
  private Command createPositionCommand(ArmPosition position) {
    return Commands.parallel(
        Commands.runOnce(() -> setPosition(position.targetAngle))
            .withName("Move to " + position.toString()),
        Commands.run(() -> checkAtTarget(position))
            .withName("Check " + position.toString() + " Target"));
  }

  /**
   * Checks if the arm is at its target position.
   *
   * @return true if at target position, false otherwise
   */
  @AutoLogOutput
  private boolean isAtTarget() {
    if (currentPosition == ArmPosition.STOWED) return true;
    Angle currentPos = getPosition();
    return Math.abs(currentPos.minus(currentPosition.targetAngle).in(Degrees))
        <= currentPosition.angleTolerance.in(Degrees);
  }

  /**
   * Logs whether the arm is at its target position for a given mode.
   *
   * @param position The position to check against
   */
  private void checkAtTarget(ArmPosition position) {
    boolean atTarget = isAtTarget();
    Logger.recordOutput("Arm/AtTarget", atTarget);
    Logger.recordOutput("Arm/TargetAngle", position.targetAngle);
  }

  /**
   * Creates a command to set the arm to a specific position.
   *
   * @param position The desired arm position
   * @return Command to set the position
   */
  public Command setPositionCommand(ArmPosition position) {
    return Commands.runOnce(() -> setArmPosition(position))
        .withName("SetArmPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to move the arm to intake position
   */
  public Command intake() {
    return setPositionCommand(ArmPosition.INTAKE);
  }

  /**
   * @return Command to move the arm to amp scoring position
   */
  public Command amp() {
    return setPositionCommand(ArmPosition.AMP);
  }

  /**
   * @return Command to move the arm to speaker scoring position
   */
  public Command speaker() {
    return setPositionCommand(ArmPosition.SPEAKER);
  }

  /**
   * @return Command to stow the arm
   */
  public Command stow() {
    return setPositionCommand(ArmPosition.STOWED);
  }

  /**
   * @return Command to stop the arm
   */
  public Command stopCommand() {
    return setPositionCommand(ArmPosition.STOP);
  }
}
