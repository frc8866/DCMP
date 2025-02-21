package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.elevatorpid;

public class AutonElevatorcmd extends Command {
  private final elevatorpid elevator;
  private final int targetPosition;
  private final double tolerance = 0.25; // Tolerance to switch from Motion Magic to PID

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
  public AutonElevatorcmd(elevatorpid elevator, int targetPosition) {
    this.elevator = elevator;
    this.targetPosition = targetPosition;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    // Start in the MOVING state and reset encoders if needed.
    currentState = State.MOVING;
    if (elevator.getLeftPosition() < 0.1) {
      elevator.resetenc();
    }

    if (elevator.checkifsetpoint1() || targetPosition == 1) {
      Constants.setElevatorState(Constants.Elevatorposition.L1);
    } else {
      Constants.setElevatorState(Constants.Elevatorposition.L0);
    }
  }

  @Override
  public void execute() {
    double currentPos = elevator.getLeftPosition();

    if (currentState == State.MOVING) {
      // Command the elevator to move using Motion Magic.
      elevator.setMotionMagic1(targetPosition);

      // When close enough to the target, switch to PID holding mode.
      if (Math.abs(currentPos - targetPosition) < tolerance) {
        currentState = State.HOLDING;
        elevator.initializePid(targetPosition);
      }
    } else if (currentState == State.HOLDING) {
      // Use PID to hold the position.
      elevator.Motionmagictoggle(targetPosition);
    }
  }

  @Override
  public boolean isFinished() {
    // This command runs until it is interrupted (for example, by another command).
    return elevator.autoncheck(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the elevator when the command ends.
    elevator.Motionmagictoggle(targetPosition);
  }
}
