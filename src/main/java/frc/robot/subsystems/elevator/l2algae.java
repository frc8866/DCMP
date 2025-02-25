  package frc.robot.subsystems.elevator;

  import edu.wpi.first.wpilibj.Timer;
  import edu.wpi.first.wpilibj2.command.Command;
  import frc.robot.subsystems.arm.algee;

  public class l2algae extends Command {
    private final algee m_algee;
    private final elevatorpid ele;
    private final double m_intakeSpeed;
    private final double m_velocityThreshold;
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
    public l2algae(
        algee algeeSubsystem,
        double intakeSpeed,
        double velocityThreshold,
        elevatorpid ele,
        double flippos) {
      this.m_algee = algeeSubsystem;
      this.m_intakeSpeed = intakeSpeed;
      this.m_velocityThreshold = velocityThreshold;
      this.ele = ele;
      this.flippos = flippos;

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

      // Once the elevator is at the intake position and the velocity is below the threshold, detect
      // the ball.
      if (ballDetected == false && ele.flipcheck(flippos) && currentVelocity < 10) {
        

        ballDetected = true;
        m_algee.setShooter(0.3);

        // Start the delay timer
        if (!timerStarted) {
          delayTimer.start();
          timerStarted = true;
        }
      }
      // After the ball is detected, wait for the delay to elapse before pulling the elevator back
      // down.
      if (ballDetected && delayTimer.get() > 1) {
        ele.setMotionMagicflip(-3.4033203125);
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
