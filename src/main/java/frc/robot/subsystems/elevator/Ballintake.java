package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.algee;

public class Ballintake extends Command {
  private final algee m_algee;
  private final double m_intakeSpeed;
  private final double m_velocityThreshold;
  private boolean ballDetected = false;

  /**
   * Constructs the BallIntakeCommand.
   *
   * @param algeeSubsystem The algee subsystem instance.
   * @param intakeSpeed The motor speed used for intaking (e.g., -0.3 for a negative percent
   *     output).
   * @param velocityThreshold The shooter velocity (absolute value) below which we assume a ball is
   *     loaded.
   */
  public Ballintake(algee algeeSubsystem, double intakeSpeed, double velocityThreshold) {
    m_algee = algeeSubsystem;
    m_intakeSpeed = intakeSpeed;
    m_velocityThreshold = velocityThreshold;
    addRequirements(m_algee);
  }

  @Override
  public void initialize() {
    ballDetected = false;
    // Start the shooter motor at the intake speed.

  }

  @Override
  public void execute() {
    // Read the current shooter velocity.
    double currentVelocity = Math.abs(m_algee.velocity());
    // If the velocity drops below the threshold, assume the ball is fully intaken.

    if (ballDetected == false) {
      m_algee.setShooter(-0.8);
    }

    if (currentVelocity < m_velocityThreshold) {
      ballDetected = true;
      m_algee.setShooter(0);
    }
  }

  @Override
  public boolean isFinished() {
    // Finish the command once the ball is detected.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Make sure the shooter motor is stopped.
    m_algee.setShooter(0);
  }
}
