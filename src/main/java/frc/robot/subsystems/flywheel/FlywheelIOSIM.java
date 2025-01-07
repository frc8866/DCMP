package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSIM extends FlywheelIOCTRE {

  private final FlywheelSim motorSimModel;
  private final TalonFXSimState leaderSim;
  private final TalonFXSimState followerSim;

  public FlywheelIOSIM() {
    super();
    leaderSim = leader.getSimState();
    followerSim = follower.getSimState();
    DCMotor motor = DCMotor.getKrakenX60Foc(2).withReduction(GEAR_RATIO);
    LinearSystem<N1, N1, N1> linearSystem =
        LinearSystemId.createFlywheelSystem(motor, 0.01, GEAR_RATIO);
    motorSimModel = new FlywheelSim(linearSystem, motor);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    super.updateInputs(inputs);
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = leaderSim.getMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    leaderSim.setRotorVelocity(
        GEAR_RATIO * Units.radiansToRotations(motorSimModel.getAngularVelocityRadPerSec()));
  }
}
