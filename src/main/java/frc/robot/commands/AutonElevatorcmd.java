package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.elevatorsub;

public class AutonElevatorcmd extends Command {
  private final elevatorsub elevator;
  private final int targetPosition;
  private final double tolerance = 0.25; // Tolerance to switch from Motion Magic to PID
  private double l0 = 0;
  private double l1 = -7.13134765625;
  private double l2 = -27.000390625;
  private double l3 = -26.700390625;
  private double l4 = -24.3123046875;

  private boolean first;
  private boolean up;

  private double flipsetpoint;
  private Timer time = new Timer();

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
  public AutonElevatorcmd(elevatorsub elevator, int targetPosition, boolean hi) {
    this.up = hi;
    this.elevator = elevator;
    this.targetPosition = targetPosition;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    time.stop();
    time.reset();

    if (targetPosition == 1) {
      Constants.setElevatorState(Constants.Elevatorposition.L1);
    } else if (targetPosition == 2) {
      Constants.setElevatorState(Constants.Elevatorposition.L2);
    } else if (targetPosition == 3) {
      Constants.setElevatorState(Constants.Elevatorposition.L3);
    } else if (targetPosition == 4) {
      Constants.setElevatorState(Constants.Elevatorposition.L4);

      // Normal
    } else {
      Constants.setElevatorState(Constants.Elevatorposition.L0);
    }
    // Start in the MOVING state and reset encoders if needed.
    currentState = State.MOVING;
    if (elevator.getLeftPosition() < 0.1) {
      elevator.resetenc();
    }

    first = true;
  }

  @Override
  public void execute() {

    if (up) {
      // Set flipsetpoint based on the desired elevator state.
      if (Constants.getElevatorState() == Constants.Elevatorposition.L1) {
        flipsetpoint = l1;

      } else if (Constants.getElevatorState() == Constants.Elevatorposition.L2) {
        flipsetpoint = l2;
      } else if (Constants.getElevatorState() == Constants.Elevatorposition.L3) {
        flipsetpoint = l3;
      } else if (Constants.getElevatorState() == Constants.Elevatorposition.L4) {
        flipsetpoint = l4;

        // BargeShoot

      }
      time.start();
      elevator.setsetpoint(flipsetpoint);

      // Check if the flip motor has reached its setpoint.
      // Note: Use flipsetpoint (not targetPosition) for the check.
      if (!elevator.flipcheck(flipsetpoint)) {

        // Command the flip motor until it is at its setpoint.
        elevator.pid();
        // Do not start moving the elevator until the flip motor is ready.
        return;
      }

      // Once the flip motor is holding its setpoint, command the elevator.

      elevator.setMotionMagic1(targetPosition);
      elevator.pid();

      // When close enough to the target, switch to PID holding mode.
      // if (Math.abs(currentPos - targetPosition) < tolerance) {
      //   currentState = State.HOLDING;
      //   elevator.initializePid(targetPosition);
      // } else if (currentState == State.HOLDING) {
      //   // Use PID to hold the position.
      //   elevator.Motionmagictoggle(targetPosition);
    } else if (up == false) {
      elevator.Motionmagic(0);
      if (elevator.check(0)) {
        elevator.setMotionMagicflip(0.6);
        // idk
      }
    }
  }

  @Override
  public boolean isFinished() {
    // This command runs until it is interrupted (for example, by another command).
    return time.get() > 3;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the elevator when the command ends.
    time.stop();
  }
}
