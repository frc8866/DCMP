package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSIM extends ElevatorIOCTRE {

  private final ElevatorSim motorSimModel;
  private final TalonFXSimState leaderSim;
  private final TalonFXSimState followerSim;
  private final CANcoderSimState encoderSim;

  public ElevatorIOSIM() {
    super();
    leaderSim = leader.getSimState();
    followerSim = follower.getSimState();
    encoderSim = leaderEncoder.getSimState();
    DCMotor motor = DCMotor.getKrakenX60Foc(2);
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createElevatorSystem(
            motor,
            Pounds.of(10).in(Kilograms), // Mass in kg
            elevatorRadius.in(Meters), // Drum radius in meters
            GEAR_RATIO);
    motorSimModel =
        new ElevatorSim(
            linearSystem,
            motor,
            GEAR_RATIO, // Add gear ratio parameter
            Feet.of(8).in(Meters), // Max height in meters
            true, // Simulate gravity
            0); // Initial position
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(leaderSim.getMotorVoltage());
    motorSimModel.update(0.020); // assume 20 ms loop time

    // Convert linear velocity to rotational velocity for the motor
    var position = Radians.of(motorSimModel.getPositionMeters() / elevatorRadius.in(Meters));
    // This is OK, since the time base is the same
    var velocity =
        RadiansPerSecond.of(motorSimModel.getVelocityMetersPerSecond() / elevatorRadius.in(Meters));

    leaderSim.setRawRotorPosition(position.times(GEAR_RATIO));
    leaderSim.setRotorVelocity(velocity.times(GEAR_RATIO));

    encoderSim.setRawPosition(position);
    encoderSim.setVelocity(velocity);
  }
}
