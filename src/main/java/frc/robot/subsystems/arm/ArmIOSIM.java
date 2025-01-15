package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSIM extends ArmIOCTRE {

  private final SingleJointedArmSim motorSimModel;
  private final TalonFXSimState leaderSim;
  private final TalonFXSimState followerSim;
  private final CANcoderSimState encoderSim;

  public ArmIOSIM() {
    super();
    leaderSim = leader.getSimState();
    followerSim = follower.getSimState();
    encoderSim = leaderEncoder.getSimState();
    DCMotor motor = DCMotor.getKrakenX60Foc(2);
    Distance armDistance = Inches.of(12);
    Mass armMass = Pounds.of(15);
    double armMOI = SingleJointedArmSim.estimateMOI(armDistance.in(Meters), armMass.in(Kilograms));
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createSingleJointedArmSystem(motor, armMOI, GEAR_RATIO);
    motorSimModel =
        new SingleJointedArmSim(
            linearSystem,
            motor,
            GEAR_RATIO,
            armDistance.in(Meters),
            Rotations.of(-30).in(Radians),
            Rotations.of(90).in(Radians),
            true,
            Rotations.of(0).in(Radians));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    super.updateInputs(inputs);
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(leaderSim.getMotorVoltage());
    motorSimModel.update(0.020); // assume 20 ms loop time

    /* Update all of our sensors. */
    Angle position = Radians.of(motorSimModel.getAngleRads());
    // This is OK, since the time base is the same
    AngularVelocity velocity = RadiansPerSecond.of(motorSimModel.getVelocityRadPerSec());
    leaderSim.setRawRotorPosition(position.times(GEAR_RATIO));
    leaderSim.setRotorVelocity(velocity.times(GEAR_RATIO));

    encoderSim.setRawPosition(position);
    encoderSim.setVelocity(velocity);
  }
}
