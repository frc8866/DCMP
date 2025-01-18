package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
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
 * The Flywheel subsystem controls a dual-motor flywheel mechanism used for shooting game pieces. It
 * supports multiple shooting modes with different target speeds and provides closed-loop control.
 */
public class Flywheel extends SubsystemBase {
  // Hardware interface and inputs
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs;

  // Current flywheel position mode
  private FlywheelPosition currentMode = FlywheelPosition.STOP;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Flywheel leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Flywheel follower motor isn't connected", AlertType.kError);

  /**
   * Creates a new Flywheel subsystem with the specified hardware interface.
   *
   * @param io The hardware interface implementation for the flywheel
   */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.inputs = new FlywheelIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // Update and log inputs from hardware
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    // Update motor connection status alerts
    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
  }

  /**
   * Runs the flywheel in closed-loop velocity mode at the specified speed.
   *
   * @param velocity The target angular velocity
   */
  private void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  /** Stops the flywheel motors. */
  private void stop() {
    io.stop();
  }

  /**
   * Returns the current velocity of the flywheel.
   *
   * @return The current angular velocity
   */
  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.leaderVelocity;
  }

  /** Enumeration of available flywheel positions with their corresponding target speeds. */
  private enum FlywheelPosition {
    STOP(RotationsPerSecond.of(0)), // Stop the flywheel
    L1(RotationsPerSecond.of(5)), // For scoring in the amp
    L2(RotationsPerSecond.of(10)), // For scoring in the speaker
    L3(RotationsPerSecond.of(20)); // For intaking/feeding game pieces

    private final AngularVelocity targetSpeed;
    private final AngularVelocity speedTolerance;

    FlywheelPosition(AngularVelocity targetSpeed, AngularVelocity speedTolerance) {
      this.targetSpeed = targetSpeed;
      this.speedTolerance = speedTolerance;
    }

    FlywheelPosition(AngularVelocity targetSpeed) {
      this(targetSpeed, RotationsPerSecond.of(1)); // 1 RPS default tolerance
    }
  }

  /**
   * Gets the current flywheel position mode.
   *
   * @return The current FlywheelPosition
   */
  public FlywheelPosition getMode() {
    return currentMode;
  }

  /**
   * Sets a new flywheel position and schedules the corresponding command.
   *
   * @param position The desired FlywheelPosition
   */
  private void setFlywheelPosition(FlywheelPosition position) {
    currentCommand.cancel();
    currentMode = position;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current position
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              FlywheelPosition.STOP,
              Commands.runOnce(this::stop).withName("Stop Flywheel"),
              FlywheelPosition.L1,
              createPositionCommand(FlywheelPosition.L1),
              FlywheelPosition.L2,
              createPositionCommand(FlywheelPosition.L2),
              FlywheelPosition.L3,
              createPositionCommand(FlywheelPosition.L3)),
          this::getMode);

  /**
   * Creates a command for a specific flywheel position that runs the flywheel and checks the target
   * speed.
   *
   * @param position The flywheel position to create a command for
   * @return A command that implements the flywheel movement
   */
  private Command createPositionCommand(FlywheelPosition position) {
    return Commands.runOnce(() -> setVelocity(position.targetSpeed))
        .withName("Run " + position.toString());
  }

  /**
   * Checks if the flywheel is at its target speed.
   *
   * @return true if at target speed, false otherwise
   */
  @AutoLogOutput
  public boolean isAtTarget() {
    if (currentMode == FlywheelPosition.STOP) return true;
    return getVelocity().isNear(currentMode.targetSpeed, currentMode.speedTolerance);
  }

  /**
   * Logs target speed for given mode.
   *
   * @return The target speed for the current mode
   */
  @AutoLogOutput
  private AngularVelocity targetSpeed() {
    return currentMode.targetSpeed;
  }

  /**
   * Creates a command to set the flywheel to a specific position.
   *
   * @param position The desired flywheel position
   * @return Command to set the position
   */
  private Command setPositionCommand(FlywheelPosition position) {
    return Commands.runOnce(() -> setFlywheelPosition(position))
        .withName("SetFlywheelPosition(" + position.toString() + ")");
  }

  /** Factory methods for common position commands */

  /**
   * @return Command to set the flywheel to AMP position
   */
  public final Command L1() {
    return setPositionCommand(FlywheelPosition.L1);
  }

  /**
   * @return Command to set the flywheel to SPEAKER position
   */
  public final Command L2() {
    return setPositionCommand(FlywheelPosition.L2);
  }

  /**
   * @return Command to set the flywheel to FEED position
   */
  public final Command L3() {
    return setPositionCommand(FlywheelPosition.L3);
  }

  /**
   * @return Command to stop the flywheel
   */
  public final Command stopCommand() {
    return setPositionCommand(FlywheelPosition.STOP);
  }
}
