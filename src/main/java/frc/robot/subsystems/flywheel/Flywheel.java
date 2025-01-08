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
 * The Flywheel subsystem controls a dual-motor flywheel mechanism used for shooting game pieces. It
 * supports multiple shooting modes with different target speeds and provides both open-loop and
 * closed-loop control options.
 */
public class Flywheel extends SubsystemBase {
  // Hardware interface and inputs
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs;

  // Alerts for motor connection status
  private final Alert leaderMotorAlert =
      new Alert("Flywheel leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Flywheel follower motor isn't connected", AlertType.kError);

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
                Logger.recordOutput("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  // Current shooting mode of the flywheel
  private ShotMode currentMode = ShotMode.NONE;

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
   * Runs the flywheel in open-loop mode at the specified voltage.
   *
   * @param volts The voltage to apply to the motors
   */
  public void runVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  /**
   * Runs the flywheel in closed-loop velocity mode at the specified speed.
   *
   * @param velocity The target angular velocity
   */
  public void runVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  /** Stops the flywheel motors. */
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
   * Returns the current velocity of the flywheel.
   *
   * @return The current angular velocity
   */
  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  /** Enumeration of available shooting modes with their corresponding target speeds. */
  public enum ShotMode {
    NONE(RotationsPerSecond.of(0)),
    AMP(RotationsPerSecond.of(5)), // For scoring in the amp
    SPEAKER(RotationsPerSecond.of(10)), // For scoring in the speaker
    FEED(RotationsPerSecond.of(20)); // For intaking/feeding game pieces

    private AngularVelocity targetSpeed;
    private AngularVelocity speedTolerance;

    ShotMode(AngularVelocity targetSpeed, AngularVelocity speedTolerance) {
      this.targetSpeed = targetSpeed;
      this.speedTolerance = speedTolerance;
    }

    ShotMode(AngularVelocity targetSpeed) {
      this(targetSpeed, RotationsPerSecond.of(1));
    }
  }

  /**
   * Gets the current shooting mode.
   *
   * @return The current ShotMode
   */
  public ShotMode getMode() {
    return currentMode;
  }

  /**
   * Sets a new shooting mode and schedules the corresponding command.
   *
   * @param mode The desired ShotMode
   */
  public void setMode(ShotMode mode) {
    currentCommand.cancel();
    currentMode = mode;
    currentCommand.schedule();
  }

  // Command that runs the appropriate routine based on the current mode
  private final Command currentCommand =
      new SelectCommand<>(
          Map.of(
              ShotMode.NONE,
                  Commands.runOnce(this::stop)
                      .alongWith(Commands.run(() -> checkAtTarget(ShotMode.NONE)))
                      .withName("Stop Flywheel"),
              ShotMode.AMP, createShotCommand(ShotMode.AMP),
              ShotMode.SPEAKER, createShotCommand(ShotMode.SPEAKER),
              ShotMode.FEED, createShotCommand(ShotMode.FEED)),
          this::getMode);

  /**
   * Creates a command for a specific shooting mode that runs the flywheel and checks the target
   * speed.
   *
   * @param mode The shooting mode to create a command for
   * @return A command that implements the shooting mode
   */
  private Command createShotCommand(ShotMode mode) {
    return Commands.parallel(
        Commands.runOnce(() -> runVelocity(mode.targetSpeed)).withName("Run " + mode.toString()),
        Commands.run(() -> checkAtTarget(mode)).withName("Check " + mode.toString() + " Target"));
  }

  /**
   * Checks if the flywheel is at its target speed.
   *
   * @return true if at target speed, false otherwise
   */
  @AutoLogOutput
  private boolean isAtTarget() {
    if (currentMode == ShotMode.NONE) return true;
    return getVelocity().isNear(currentMode.targetSpeed, currentMode.speedTolerance);
  }

  /**
   * Logs whether the flywheel is at its target speed for a given mode.
   *
   * @param mode The mode to check against
   */
  private void checkAtTarget(ShotMode mode) {
    boolean atTarget = isAtTarget();
    Logger.recordOutput("Flywheel/AtTarget", atTarget);
    Logger.recordOutput("Flywheel/TargetSpeed", mode.targetSpeed);
  }

  /**
   * Creates a command to set the flywheel to a specific mode.
   *
   * @param mode The desired shot mode
   * @return Command to set the mode
   */
  public Command setModeCommand(ShotMode mode) {
    return Commands.runOnce(() -> setMode(mode))
        .withName("SetFlywheelMode(" + mode.toString() + ")");
  }

  /**
   * Factory methods for common mode commands. These make binding to buttons/triggers more concise.
   */

  /**
   * @return Command to set the flywheel to AMP mode
   */
  public Command amp() {
    return setModeCommand(ShotMode.AMP);
  }

  /**
   * @return Command to set the flywheel to SPEAKER mode
   */
  public Command speaker() {
    return setModeCommand(ShotMode.SPEAKER);
  }

  /**
   * @return Command to set the flywheel to FEED mode
   */
  public Command feed() {
    return setModeCommand(ShotMode.FEED);
  }

  /**
   * @return Command to stop the flywheel
   */
  public Command stopCommand() {
    return setModeCommand(ShotMode.NONE);
  }
}
