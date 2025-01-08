package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSIM extends FlywheelIOCTRE {

  private final FlywheelSim motorSimModel;
  private final TalonFXSimState leaderSim;

  Distance radius = Inches.of(1.5);
  double moi = Pounds.of(8.0).in(Kilograms) * Math.pow(radius.in(Meters), 2);

  public FlywheelIOSIM() {
    super();

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kS = 0.01;
    config.Slot0.kV = 0.082;
    config.Slot0.kA = 0;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    leaderSim = leader.getSimState();
    DCMotor motor = DCMotor.getKrakenX60Foc(2);
    LinearSystem<N1, N1, N1> linearSystem =
        LinearSystemId.createFlywheelSystem(motor, moi, GEAR_RATIO);
    motorSimModel = new FlywheelSim(linearSystem, motor);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    super.updateInputs(inputs);
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = leaderSim.getMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    leaderSim.setRotorVelocity(motorSimModel.getAngularVelocity());
    leaderSim.addRotorPosition(motorSimModel.getAngularVelocity().in(RotationsPerSecond) * 0.02);
  }
}
