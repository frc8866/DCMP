package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Abstract base class for mechanisms that have discrete states and closed-loop control. This class
 * provides common functionality for subsystems like arms and flywheels.
 *
 * @param <T> The type of the target value (e.g., Angle for arm, AngularVelocity for flywheel)
 * @param <S> The enum type representing states (must implement TargetProvider<T>)
 */
public abstract class ControlledMechanism<
        T, S extends Enum<S> & ControlledMechanism.TargetProvider<T>>
    extends SubsystemBase {

  /** Interface for enums that provide target values */
  public interface TargetProvider<T> {
    T getTarget();

    T getTolerance();
  }

  // Current state
  private S currentState;

  /**
   * Creates a new ControlledMechanism with the specified initial state.
   *
   * @param initialState The initial state
   */
  protected ControlledMechanism(S initialState) {
    this.currentState = initialState;
  }

  @Override
  public void periodic() {
    updateInputs();
    updateAlerts();
  }

  protected abstract void updateInputs();

  protected abstract void updateAlerts();

  /**
   * Sets the mechanism to the specified target value
   *
   * @param target The target value to achieve
   */
  protected abstract void setTarget(T target);

  /** Stops the mechanism */
  protected abstract void stop();

  /**
   * Gets the current measured value
   *
   * @return The current value
   */
  @AutoLogOutput
  public abstract T getCurrentValue();

  /**
   * Gets the current state.
   *
   * @return The current state
   */
  public S getState() {
    return currentState;
  }

  /**
   * Sets a new state and schedules the corresponding command.
   *
   * @param state The desired state
   */
  private void setState(S state) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    currentState = state;
    currentCommand.schedule();
  }

  protected abstract Map<S, Command> createCommandMap();

  // Command that runs the appropriate routine based on the current state
  private final Command currentCommand = new SelectCommand<>(createCommandMap(), this::getState);

  /**
   * Creates a command for a specific target value.
   *
   * @param target The target value to achieve
   * @return A command that implements the movement/control
   */
  protected Command createTargetCommand(T target) {
    return Commands.runOnce(() -> setTarget(target)).withName("Set target to " + target.toString());
  }

  /**
   * Checks if the mechanism is at its target value.
   *
   * @return true if at target, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (isStopState(currentState)) return true;
    return isNear(getCurrentValue(), currentState.getTarget(), currentState.getTolerance());
  }

  /**
   * Checks if the given state is the "stop" state for this mechanism.
   *
   * @param state The state to check
   * @return true if it's the stop state, false otherwise
   */
  protected abstract boolean isStopState(S state);

  /**
   * Checks if two values are within tolerance of each other.
   *
   * @param current The current value
   * @param target The target value
   * @param tolerance The acceptable tolerance
   * @return true if within tolerance, false otherwise
   */
  protected abstract boolean isNear(T current, T target, T tolerance);

  /**
   * Creates a command to set the mechanism to a specific state.
   *
   * @param state The desired state
   * @return Command to set the state
   */
  protected Command createStateCommand(S state) {
    return Commands.runOnce(() -> setState(state))
        .withName("Set" + getClass().getSimpleName() + "State(" + state.toString() + ")");
  }
}
