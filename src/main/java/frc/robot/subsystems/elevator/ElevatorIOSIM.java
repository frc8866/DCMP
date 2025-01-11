package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

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

  public ElevatorIOSIM() {
    super();
    leaderSim = leader.getSimState();
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

    // get the motor voltage of the TalonFX
    var motorVoltage = leaderSim.getMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    // Convert linear velocity to rotational velocity for the motor
    double circumference = elevatorRadius.times(2 * Math.PI).in(Meters);
    leaderSim.setRotorVelocity(
        motorSimModel.getVelocityMetersPerSecond() / circumference * GEAR_RATIO);
    leaderSim.setRawRotorPosition(motorSimModel.getPositionMeters() / circumference * GEAR_RATIO);
  }
}
