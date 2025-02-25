package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.algee;

public class l3algae extends Command {
  private final algee m_algee;
  private final elevatorpid ele;
  private final double m_intakeSpeed;
  private final double m_velocityThreshold;
  private final double finalFlipPos;

  // Define the elevator up setpoint (adjust as needed)
  private final double elevatorUpSetpoint = 7.75048828125;

  private boolean ballDetected = false;
  private int state = 0;

  /**
   * Constructs the Ballintake2 command.
   *
   * @param algeeSubsystem The algee subsystem instance.
   * @param intakeSpeed The motor speed used for intaking (e.g., -0.3 for a negative percent
   *     output).
   * @param velocityThreshold The shooter velocity threshold (absolute value) below which we assume
   *     a ball is loaded.
   * @param ele The elevator subsystem instance.
   * @param flippos The final flip mechanism setpoint.
   */
  public l3algae(
      algee algeeSubsystem,
      double intakeSpeed,
      double velocityThreshold,
      elevatorpid ele,
      double flippos) {
    this.m_algee = algeeSubsystem;
    this.m_intakeSpeed = intakeSpeed;
    this.m_velocityThreshold = velocityThreshold;
    this.ele = ele;
    this.finalFlipPos = flippos;
    addRequirements(m_algee);
    // Optionally, add requirements for the elevator if needed:
    // addRequirements(ele);
  }

  @Override
  public void initialize() {
    ballDetected = false;
    state = 0;
  }

  @Override
  public void execute() {
    double currentVelocity = Math.abs(m_algee.velocity());

    switch (state) {
      case 0:
        // Stage 0: Pivot the flip mechanism to -17 and run shooter at intake speed.
        ele.setMotionMagicflip(-17);
        m_algee.setShooter(m_intakeSpeed);
        // When the flip mechanism reaches -17 and shooter velocity indicates ball detection,
        // transition.
        if (ele.flipcheck(-17) && currentVelocity < m_velocityThreshold) {
          ballDetected = true;
          state = 1;
        }
        break;

      case 1:
        // Stage 1: Command the elevator to go up while running shooter at a constant speed.
        ele.setMotionMagic(elevatorUpSetpoint);
        m_algee.setShooter(0.3);
        // Transition when the elevator reaches its setpoint.
        if (ele.check(elevatorUpSetpoint)) {
          state = 2;
        }
        break;

      case 2:
        // Stage 2: Command the flip mechanism to move to the final flip position.
        ele.setMotionMagicflip(finalFlipPos);
        break;

      default:
        break;
    }
  }

  @Override
  public boolean isFinished() {
    // Modify this if you want the command to end at a certain stage.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_algee.setShooter(0);
  }
}
