package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.algee;
import frc.robot.subsystems.elevator.elevatorsub;

public class l3algaeCurrent extends Command {
  private final algee m_algee;
  private final elevatorsub ele;
  private final double m_intakeSpeed;
  private final double m_currentThreshold; // current threshold (in amps) to detect a ball
  private final double elevatorpos; // elevator setpoint for intake
  private final double flippos; // flip mechanism (intake) setpoint

  private boolean ballDetected = false;
  private double currentAmps;
  private final Timer delayTimer = new Timer();
  private boolean timerStarted = false;

  /**
   * Constructs the l3algaeCurrent command.
   *
   * @param algeeSubsystem The algee subsystem instance.
   * @param intakeSpeed The motor speed used for intaking (e.g., -0.3 for a negative percent
   *     output).
   * @param currentThreshold The current threshold (in amps) above which we assume a ball is loaded.
   * @param ele The elevator subsystem instance.
   * @param flippos The flip mechanism (intake) setpoint.
   * @param elevatorpos The elevator setpoint for intake.
   */
  public l3algaeCurrent(
      algee algeeSubsystem,
      double intakeSpeed,
      double currentThreshold,
      elevatorsub ele,
      double flippos,
      double elevatorpos) {
    this.m_algee = algeeSubsystem;
    this.m_intakeSpeed = intakeSpeed;
    this.m_currentThreshold = currentThreshold;
    this.ele = ele;
    this.flippos = flippos;
    this.elevatorpos = elevatorpos;
    addRequirements(m_algee, ele);
  }

  @Override
  public void initialize() {
    delayTimer.stop();
    delayTimer.reset();
    ballDetected = false;
    timerStarted = false;
    System.out.println("l3algaeCurrent: Initialized");
  }

  @Override
  public void execute() {
    // Get the current draw from the shooter (in amps)
    currentAmps = m_algee.getCurrent();
    System.out.println("l3algaeCurrent: Shooter Current = " + currentAmps + " A");

    // Command the flip mechanism to the intake position and run the shooter at intake speed until
    // it's in position.
    if (!ballDetected && !ele.flipcheck(flippos)) {
      System.out.println("l3algaeCurrent: Moving flip to intake position: " + flippos);
      ele.setMotionMagicflip(flippos);
      m_algee.setShooter(m_intakeSpeed);
    }
    // Once the flip is in position, if the current spike indicates a ball load, detect the ball.
    else if (!ballDetected && ele.flipcheck(flippos) && currentAmps > m_currentThreshold) {
      System.out.println("l3algaeCurrent: Ball detected (current > " + m_currentThreshold + " A).");
      ballDetected = true;
      // Set shooter to a continuous speed after detection
      m_algee.setShooter(0.5);
      // Command the elevator to move to its intake setpoint
      ele.setMotionMagic(elevatorpos);
      if (!timerStarted) {
        delayTimer.start();
        timerStarted = true;
        System.out.println("l3algaeCurrent: Delay timer started.");
      }
    }

    // After the ball is detected, wait for the delay to elapse before resetting the mechanisms.
    if (ballDetected && delayTimer.get() > 1) {
      System.out.println("l3algaeCurrent: Delay elapsed. Resetting flip and elevator.");
      ele.setMotionMagicflip(-3);
      ele.setMotionMagic(0);
    }

    // If the ball is not detected, ensure the timer is reset.
    if (!ballDetected) {
      delayTimer.stop();
      delayTimer.reset();
      timerStarted = false;
    }
  }

  @Override
  public boolean isFinished() {
    // The command continues running. Adjust this condition if you need the command to end.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("l3algaeCurrent: Command ended.");
  }
}
