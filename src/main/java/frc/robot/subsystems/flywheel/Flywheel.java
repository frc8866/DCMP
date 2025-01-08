// Copyright FRC 5712
// https://hemlock5712.github.io/
//
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

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs;

  private final Alert leaderMotorAlert =
      new Alert("Flywheel leader motor isn't connected", AlertType.kError);
  private final Alert followerMotorAlert =
      new Alert("Flywheel follower motor isn't connected", AlertType.kError);

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

  private ShotMode currentMode = ShotMode.NONE;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.inputs = new FlywheelIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    leaderMotorAlert.set(!inputs.leaderConnected);
    followerMotorAlert.set(!inputs.followerConnected);
  }

  /** Run open loop at the specified voltage. */
  public void runVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  public void runVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public enum ShotMode {
    NONE(RotationsPerSecond.of(0)),
    AMP(RotationsPerSecond.of(5)), // Example target speeds - adjust as needed
    SPEAKER(RotationsPerSecond.of(10)),
    FEED(RotationsPerSecond.of(20));

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

  public ShotMode getMode() {
    return currentMode;
  }

  public void setMode(ShotMode mode) {
    currentCommand.cancel();
    currentMode = mode;
    currentCommand.schedule();
  }

  // Command to run the flywheel in the current mode
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

  private Command createShotCommand(ShotMode mode) {
    return Commands.parallel(
        // The main command that runs the flywheel
        Commands.runOnce(() -> runVelocity(mode.targetSpeed)).withName("Run " + mode.toString()),
        // The parallel command that continuously checks if we're at target
        Commands.run(() -> checkAtTarget(mode)).withName("Check " + mode.toString() + " Target"));
  }

  @AutoLogOutput
  private boolean isAtTarget() {
    if (currentMode == ShotMode.NONE) return true;
    return getVelocity().isNear(currentMode.targetSpeed, currentMode.speedTolerance);
  }

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
  public Command amp() {
    return setModeCommand(ShotMode.AMP);
  }

  public Command speaker() {
    return setModeCommand(ShotMode.SPEAKER);
  }

  public Command feed() {
    return setModeCommand(ShotMode.FEED);
  }

  public Command stopCommand() {
    return setModeCommand(ShotMode.NONE);
  }
}
