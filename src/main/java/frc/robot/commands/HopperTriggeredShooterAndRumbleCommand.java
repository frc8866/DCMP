package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.flywheel.shooter;
import frc.robot.utils.TunableController;

public class HopperTriggeredShooterAndRumbleCommand extends Command {
  private final shooter shooterSubsystem;
  private final TunableController controller;
  private final Timer timer = new Timer();
  private final Timer waittTimer = new Timer();

  // Flag indicating the ball has triggered the event.
  private boolean triggered = false;

  // Distance threshold in mm (adjust as needed)
  private final double distanceThreshold = 75.0;

  // Duration for which the shooter will run at 20% (in seconds)
  private final double burstDuration = 0.1;

  // Duration for the controller rumble (in seconds)
  private final double rumbleDuration = 2.0;

  /**
   * Constructs a new HopperTriggeredShooterAndRumbleCommand.
   *
   * @param shooterSubsystem The shooter subsystem (must provide getDistance(), setShooter(), and setHopper()).
   * @param controller       The XboxController used for rumble feedback.
   */
  public HopperTriggeredShooterAndRumbleCommand(shooter shooterSubsystem, TunableController controller) {
    this.shooterSubsystem = shooterSubsystem;
    this.controller = controller;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    // Start by running the shooter at 7% and the hopper at 25%.
    
    triggered = false;
  }

  @Override
  public void execute() {
    // If not yet triggered and the LaserCan reports the ball is at or below the threshold:
    if(shooterSubsystem.check()){
      shooterSubsystem.setShooter(0.07, 0.2);


    }

    else if(shooterSubsystem.getDistance() < 75) {
      // Trigger the event.
      triggered = true;
      // Start the timer.
      waittTimer.start();
      // Run the shooter at 20% and the hopper at 100%.
      if (waittTimer.get() < 0.1) {
        shooterSubsystem.setShooter(0.2, 0);
        
      
      }
      else if(waittTimer.get()>0.1){
        timer.start();
        shooterSubsystem.setShooter(0, 0);
        if (timer.get()<2) {
          controller.setRumble(RumbleType.kBothRumble, 1);
          
        }
        
      }
      // Start the rumble.
      
    } 

    
      
    
      
    
  }

  @Override
  public boolean isFinished() {
    // The command finishes once the rumble period (2 seconds) has elapsed after triggering.
    return timer.get()>2;
  }

  @Override
  public void end(boolean interrupted) {
    // Ensure all motors are stopped.
   shooterSubsystem.setShooter(0, 0);
    // Stop the rumble.
    controller.setRumble(XboxController.RumbleType.kBothRumble, 0.0);
    timer.stop();
    timer.reset();
    waittTimer.stop();
    waittTimer.reset();
  }
}
