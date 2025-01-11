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
import frc.robot.utils.ControlledMechanism;
import frc.robot.utils.ControlledMechanism.TargetProvider;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * The Arm subsystem controls a dual-motor arm mechanism for game piece manipulation. It supports
 * multiple positions for different game actions and provides both open-loop and closed-loop control
 * options.
 */
public class Arm extends ControlledMechanism<Angle, Arm.ArmPosition> {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs;
  private final Alert leaderMotorAlert;
  private final Alert followerMotorAlert;
  private final Alert encoderAlert;

  /** Enumeration of available arm positions with their corresponding target angles. */
  public enum ArmPosition implements TargetProvider<Angle> {
    STOP(Degrees.of(0)),
    INTAKE(Degrees.of(0)),
    L1(Degrees.of(90)),
    L2(Degrees.of(135)),
    L3(Degrees.of(135)),
    L4(Degrees.of(180));

    private final Angle targetAngle;
    private final Angle angleTolerance;

    ArmPosition(Angle targetAngle, Angle angleTolerance) {
      this.targetAngle = targetAngle;
      this.angleTolerance = angleTolerance;
    }

    ArmPosition(Angle targetAngle) {
      this(targetAngle, Degrees.of(2));
    }

    @Override
    public Angle getTarget() {
      return targetAngle;
    }

    @Override
    public Angle getTolerance() {
      return angleTolerance;
    }
  }

  public Arm(ArmIO io) {
    super(ArmPosition.INTAKE); // Set initial position
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();

    this.leaderMotorAlert = new Alert("Arm leader motor isn't connected", AlertType.kError);
    this.followerMotorAlert = new Alert("Arm follower motor isn't connected", AlertType.kError);
    this.encoderAlert = new Alert("Arm encoder isn't connected", AlertType.kError);
  }

  @Override
  protected void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  @Override
  protected void updateAlerts() {
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
    encoderAlert.set(!inputs.encoderConnected);
  }

  @Override
  protected void setTarget(Angle target) {
    io.setPosition(target);
  }

  @Override
  protected void stop() {
    io.stop();
  }

  @Override
  public Angle getCurrentValue() {
    return inputs.encoderPosition;
  }

  @Override
  protected Map<ArmPosition, Command> createCommandMap() {
    return Map.of(
        ArmPosition.STOP, Commands.runOnce(this::stop).withName("Stop Arm"),
        ArmPosition.INTAKE, createStateCommand(ArmPosition.INTAKE),
        ArmPosition.L1, createStateCommand(ArmPosition.L1),
        ArmPosition.L2, createStateCommand(ArmPosition.L2),
        ArmPosition.L3, createStateCommand(ArmPosition.L3),
        ArmPosition.L4, createStateCommand(ArmPosition.L4));
  }

  @Override
  protected boolean isStopState(ArmPosition mode) {
    return mode == ArmPosition.STOP;
  }

  @Override
  protected boolean isNear(Angle current, Angle target, Angle tolerance) {
    return current.isNear(target, tolerance);
  }

  // Factory methods remain the same
  public final Command L1() {
    return createStateCommand(ArmPosition.L1);
  }

  /**
   * @return Command to move the arm to L2 scoring position
   */
  public final Command L2() {
    return createStateCommand(ArmPosition.L2);
  }

  /**
   * @return Command to move the arm to L3 position
   */
  public final Command L3() {
    return createStateCommand(ArmPosition.L3);
  }

  /**
   * @return Command to move the arm to L4 position
   */
  public final Command L4() {
    return createStateCommand(ArmPosition.L4);
  }

  /**
   * @return Command to intake the arm
   */
  public final Command intake() {
    return createStateCommand(ArmPosition.INTAKE);
  }

  /**
   * @return Command to stop the arm
   */
  public final Command stopCommand() {
    return createStateCommand(ArmPosition.STOP);
  }
}
