package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.algee;
import frc.robot.subsystems.elevator.elevatorsub;

public class Flippy extends Command {
  private final elevatorsub elevator;
  private double flipyposition;
  private final double targetPosition;
  private algee algee;

  private enum State {
    MOVING,
    HOLDING
  }

  private State currentState;

  /**
   * Creates a new ElevatorMoveAndHoldCommand.
   *
   * @param elevator The elevator subsystem.
   * @param targetPosition The target position (in sensor units) to move to.
   */
  public Flippy(elevatorsub elevator, double targetPosition, double flipyposition) {
    this.elevator = elevator;
    this.targetPosition = targetPosition;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!elevator.flipcheck(flipyposition)) {
      // Command the flip motor until it is at its setpoint.
      elevator.setMotionMagicflip(flipyposition);
      // Do not start moving the elevator until the flip motor is ready.
      return;
    }
    elevator.setMotionMagic(targetPosition);
    elevator.setMotionMagicflip(flipyposition);
  }

  @Override
  public boolean isFinished() {
    // This command runs until it is interrupted (for example, by another command).
    return elevator.autoncheckposition(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the elevator when the command ends.

  }
}
