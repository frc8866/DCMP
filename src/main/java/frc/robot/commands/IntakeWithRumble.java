package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.shooter;
import frc.robot.utils.TunableController;

public class IntakeWithRumble extends Command {
  private final shooter shooterSubsystem;
  private final TunableController controller;
  private final Timer rumbleTimer = new Timer();
  private final double intakeSpeed; // store the intake speed parameter
  private boolean rumbleStarted = false;
  // Flag to ensure we run initialization code once.
  private boolean didInit = false;

  // Adjust these values as needed.
  private static final double CURRENT_SPIKE_THRESHOLD =
      40.0; // amps (used in shooter.hasCurrentSpike())
  private static final double RUMBLE_DURATION = 0.5; // seconds

  public IntakeWithRumble(
      shooter shooterSubsystem, TunableController controller, double intakeSpeed) {
    this.shooterSubsystem = shooterSubsystem;
    this.controller = controller;
    this.intakeSpeed = intakeSpeed; // store the parameter for later use
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    rumbleTimer.stop();
    rumbleTimer.reset();

    // Run one-time setup code here.
    rumbleStarted = false;
  }

  @Override
  public void execute() {
    if (shooterSubsystem.hasCurrentSpike() == false) {
      shooterSubsystem.speed(intakeSpeed);
    }

    if (shooterSubsystem.hasCurrentSpike() == true) {
      shooterSubsystem.speed(0);
    }
    if (shooterSubsystem.hasCurrentSpike() && !rumbleStarted) {
      rumbleStarted = true;
      rumbleTimer.start();
      controller.setRumble(RumbleType.kBothRumble, 1);
    }
    if (rumbleStarted && rumbleTimer.hasElapsed(3)) {
      controller.setRumble(RumbleType.kBothRumble, 0);
      rumbleStarted = false;
    }
  }

  @Override
  public boolean isFinished() {
    // End the command once the ball has been intaken and we've completed the rumble.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    rumbleTimer.stop();
    rumbleTimer.reset();
    // Ensure that we stop both the motor and the rumble when the command ends.
    shooterSubsystem.speed(-0.1);
    // If a current spike was detected, trigger the rumble.
    controller.setRumble(RumbleType.kBothRumble, 0);
  }
}
