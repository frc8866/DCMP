package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.utils.Conversions;

public class ElevatorIOSIM extends ElevatorIOCTRE {

  private final ElevatorSim motorSimModel;
  private final TalonFXSimState leaderSim;
  private final TalonFXSimState followerSim;
  private final CANcoderSimState encoderSim;

  public ElevatorIOSIM() {
    super();
    leaderSim = leader.getSimState();
    followerSim = follower.getSimState();
    encoderSim = encoder.getSimState();
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
    Angle position =
        Conversions.metersToRotations(
            Meters.of(motorSimModel.getPositionMeters()), GEAR_RATIO, elevatorRadius);
    // This is OK, since the time base is the same
    AngularVelocity velocity =
        Conversions.metersToRotationsVel(
            MetersPerSecond.of(motorSimModel.getVelocityMetersPerSecond()),
            GEAR_RATIO,
            elevatorRadius);

    leaderSim.setRawRotorPosition(position);
    leaderSim.setRotorVelocity(velocity);

    encoderSim.setRawPosition(position.times(GEAR_RATIO));
    encoderSim.setVelocity(velocity.times(GEAR_RATIO));
  }
}
