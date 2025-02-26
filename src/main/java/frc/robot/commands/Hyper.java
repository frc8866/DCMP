package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.algee;
import frc.robot.subsystems.elevator.elevatorsub;

public class Hyper extends Command {
  private final algee m_algee;
  private final elevatorsub ele;
  private final double m_intakeSpeed;
  private final double m_velocityThreshold;
  private double elevatorpos;

  private boolean ballDetected = false;
  private double currentVelocity;
  private double flippos;
  private Timer delayTimer = new Timer();
  private boolean timerStarted = false;

  /**
   * Constructs the BallIntakeCommand.
   *
   * @param algeeSubsystem The algee subsystem instance.
   * @param intakeSpeed The motor speed used for intaking (e.g., -0.3 for a negative percent
   *     output).
   * @param velocityThreshold The shooter velocity (absolute value) below which we assume a ball is
   *     loaded.
   */
  public Hyper(
      algee algeeSubsystem,
      double intakeSpeed,
      double velocityThreshold,
      elevatorsub ele,
      double flippos,
      double elevatorpos) {
    this.m_algee = algeeSubsystem;
    this.m_intakeSpeed = intakeSpeed;
    this.m_velocityThreshold = velocityThreshold;
    this.ele = ele;
    this.flippos = flippos;
    this.elevatorpos = elevatorpos;

    addRequirements(m_algee);
  }

  @Override
  public void initialize() {
    delayTimer.stop();
    delayTimer.reset();
    ballDetected = false;
    timerStarted = false;
    // Start the shooter motor at the intake speed.

  }

  @Override
  public void execute() {

    // Read the current shooter velocity.
    currentVelocity = Math.abs(m_algee.velocity());
    // If the ball is not yet detected and the elevator hasn't reached the intake position, command
    // the elevator to move there.
    if (!ballDetected && !ele.flipcheck(flippos)) {
      ele.setMotionMagicflip(flippos);
      m_algee.setShooter(m_intakeSpeed);
    }
    if (ballDetected == false && ele.flipcheck(flippos) && currentVelocity > 10) {
      ele.setMotionMagicflip(flippos);
      m_algee.setShooter(m_intakeSpeed);

      ele.setMotionMagic(elevatorpos);
    }

    // Once the elevator is at the intake position and the velocity is below the threshold, detect
    // the ball.
    if (ballDetected == false && ele.flipcheck(flippos) && currentVelocity < 10) {

      ballDetected = true;
      m_algee.setShooter(0.3);
      ele.setMotionMagic(elevatorpos);

      // Start the delay timer
      if (!timerStarted) {
        delayTimer.start();
        timerStarted = true;
      }
    }
    // After the ball is detected, wait for the delay to elapse before pulling the elevator back
    // down.
    if (ballDetected && delayTimer.get() > 1) {
      ele.setMotionMagicflip(-17);
      ele.setMotionMagic(5);
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

  }
}
