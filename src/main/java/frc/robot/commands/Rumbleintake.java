package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.shooter;
import frc.robot.utils.TunableController;

public class Rumbleintake extends Command {
  private final shooter shoot;
  private final TunableController controller;
  private final Timer timer = new Timer();
  private boolean hasRumbled = false;

  /**
   * Constructs the command.
   *
   * @param motorSubsystem The subsystem controlling the Krakcen motor.
   * @param controller The controller to be rumbled.
   */
  public Rumbleintake(shooter motorSubsystem, TunableController controller) {
    this.shoot = motorSubsystem;
    this.controller = controller;
    addRequirements(motorSubsystem);
  }

  @Override
  public void initialize() {
    // Reset and start the timer when the command is initialized.
    timer.reset();
    timer.start();
    hasRumbled = false;
  }

  @Override
  public void execute() {
    // Check if the motor's velocity is below 30 and we haven't already started the rumble.
    if (!hasRumbled && shoot.velocity() < 30) {
      // Start the rumble on both sides of the controller.
      controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
      hasRumbled = true;
    }
  }

  @Override
  public boolean isFinished() {
    // End the command after 2 seconds have elapsed since starting the rumble.
    return hasRumbled && timer.get() >= 2.0;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the rumble regardless of how the command ended.
    controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    timer.stop();
  }
}
