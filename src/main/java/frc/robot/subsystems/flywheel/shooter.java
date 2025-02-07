package frc.robot.subsystems.flywheel;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
// import frc.robot.subsystems.lookuptable.setpoint;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  private TalonFX lshoot = new TalonFX(17);
  private TalonFX rshoot = new TalonFX(18);
  private LaserCan lc = new LaserCan(0);
  private TalonFX hopper = new TalonFX(30);
  private int  distance;

  public shooter() {

    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void periodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putNumber("distance", measurement.distance_mm);
      distance = measurement.distance_mm;
      
    } else {
      System.out.println(
          "Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an
      // unreliable measurement.
    }
  }

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

  public Command both(double sspeed, double hspeed) {
    return new Command() {
      @Override
      public void initialize() {
        // Initialization code, such as resetting encoders or PID controllers
      }

      @Override
      public void execute() {

        if (distance > 39) {
            lshoot.set(sspeed);
          rshoot.set(-sspeed);
          hopper.set(hspeed);
          
        }
        else{
          lshoot.set(0);
          rshoot.set(0);
          hopper.set(0);
        }

      }

      @Override
      public void end(boolean interrupted) {
        hopper.set(0);
        lshoot.set(0);
        rshoot.set(0);
      }

      @Override
      public boolean isFinished() {
        return distance > 39; // 39 is a setpoint number i need to find the acc number
      }
    };
  }

  public Command hopper(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        // Initialization code, such as resetting encoders or PID controllers
      }

      @Override
      public void execute() {

        hopper.set(speed / 100);
      }

      @Override
      public void end(boolean interrupted) {
        hopper.set(0);
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
