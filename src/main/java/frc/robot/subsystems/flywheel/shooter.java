package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
// import frc.robot.subsystems.lookuptable.setpoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  private TalonFX lshoot = new TalonFX(17);
  private TalonFX rshoot = new TalonFX(18);

  public shooter() {}

  @Override
  public void periodic() {}

  public Command cmd(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        // Initialization code, such as resetting encoders or PID controllers
      }

      @Override
      public void execute() {

        lshoot.set(speed / 100);
        rshoot.set(-speed / 100);
      }

      @Override
      public void end(boolean interrupted) {
        lshoot.set(0);
        rshoot.set(0);
      }

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command cmd3(double speed, double speed2) {
    return new Command() {
      @Override
      public void initialize() {
        // Initialization code, such as resetting encoders or PID controllers
      }

      @Override
      public void execute() {

        lshoot.set(speed / 100);
        rshoot.set(-speed2 / 100);
      }

      @Override
      public void end(boolean interrupted) {
        lshoot.set(0);
        rshoot.set(0);
      }

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }
}
