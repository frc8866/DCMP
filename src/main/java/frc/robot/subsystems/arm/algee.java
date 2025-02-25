package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class algee extends SubsystemBase {
  private TalonFX algaeshooter = new TalonFX(15);
  TalonFXConfiguration cfg = new TalonFXConfiguration();
  private Slot0Configs slot0 = cfg.Slot0;
  MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private PIDController pid = new PIDController(0.07, 0, 0);

  public algee() {

    slot0.kS = 0.25;
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 0.3; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0; // no output for integrated error
    slot0.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        200; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)\

    algaeshooter.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Velocity", algaeshooter.getVelocity().getValueAsDouble());
  }

  public double velocity() {
    return algaeshooter.getVelocity().getValueAsDouble();
  }

  public void setShooter(double speed) {
    algaeshooter.set(speed);
  }

  // public void speed(double position) {
  //   pid.setSetpoint(position);

  //   double speed = pid.calculate(algaepiv.getPosition().getValueAsDouble());
  //   algaepiv.set(speed);
  // }

  public Command algeacmd(double position, double speed) {
    return new Command() {
      @Override
      public void initialize() {
        pid.setSetpoint(position);
        // Initialization code, such as resetting encoders or PID controllers
      }

      @Override
      public void execute() {}

      @Override
      public void end(boolean interrupted) {
        algaeshooter.set(0);
      }

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached; // Check if the setpoint is reached
      }
    };
  }

  // public Command position(double position) {
  //   return new Command() {
  //     @Override
  //     public void initialize() {
  //       pid.setSetpoint(position);
  //       // Initialization code, such as resetting encoders or PID controllers
  //     }

  //     @Override
  //     public void execute() {

  //       double speed1 = pid.calculate(algaepiv.getPosition().getValueAsDouble());
  //       algaepiv.set(speed1);
  //     }

  //     @Override
  //     public void end(boolean interrupted) {}

  //     @Override
  //     public boolean isFinished() {
  //       return false; // Check if the setpoint is reached; // Check if the setpoint is reached
  //     }
  //   };
  // }

  // public Command ion(double speed) {
  //   return new Command() {
  //     @Override
  //     public void initialize() {
  //       // Initialization code, such as resetting encoders or PID controllers
  //     }

  //     @Override
  //     public void execute() {
  //       algaepiv.set(speed);
  //     }

  //     @Override
  //     public void end(boolean interrupted) {
  //       algaeshooter.set(0);
  //     }

  //     @Override
  //     public boolean isFinished() {
  //       return false; // Check if the setpoint is reached; // Check if the setpoint is reached
  //     }
  //   };
  // }
}
