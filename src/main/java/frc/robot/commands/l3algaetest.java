package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.algee;
import frc.robot.subsystems.elevator.elevatorsub;

public class l3algaetest extends Command {
  private final algee m_algee;
  private final elevatorsub ele;
  private final double m_intakeSpeed;
  private final double m_velocityThreshold;
  private double elevatorpos;
  private boolean l2;

  private boolean ballDetected = false;
  private double currentVelocity;
  private double flippos;
  private Timer delayTimer = new Timer();
  private boolean timerStarted = false;
  private double position;
  private int epos;

  /**
   * Constructs the BallIntakeCommand.
   *
   * @param algeeSubsystem The algee subsystem instance.
   * @param intakeSpeed The motor speed used for intaking (e.g., -0.3 for a negative percent
   *     output).
   * @param velocityThreshold The shooter velocity (absolute value) below which we assume a ball is
   *     loaded.
   */
  public l3algaetest(
      algee algeeSubsystem,
      double intakeSpeed,
      double velocityThreshold,
      elevatorsub ele,
      double flippos,
      double elevatorpos,
      boolean l2) {
    this.m_algee = algeeSubsystem;
    this.m_intakeSpeed = intakeSpeed;
    this.m_velocityThreshold = velocityThreshold;
    this.ele = ele;
    this.flippos = flippos;
    this.elevatorpos = elevatorpos;
    this.l2 = l2;

    addRequirements(m_algee);
  }

  @Override
  public void initialize() {
    delayTimer.stop();
    delayTimer.reset();
    ballDetected = false;
    timerStarted = false;
    // Start the shooter motor at the intake speed.
    epos = 0;
  }

  @Override
  public void execute() {

    if (l2) {
      position = -27;
      epos = 2;

    } else {
      position = -27;
      epos = 3;
    }

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
      m_algee.setShooter(0.4);
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
      ele.setposition(position);

      // Check if the flip motor has reached its setpoint.
      // Note: Use flipsetpoint (not targetPosition) for the check.
      if (!ele.flipcheck(position)) {

        // Command the flip motor until it is at its setpoint.
        ele.pid();
        // Do not start moving the elevator until the flip motor is ready.
        return;
      }

      // Once the flip motor is holding its setpoint, command the elevator.

      ele.setMotionMagic1(epos);
      ele.pid();
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
