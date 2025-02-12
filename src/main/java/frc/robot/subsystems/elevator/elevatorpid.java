package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.*;
import java.util.List;

public class elevatorpid extends SubsystemBase {
  private List<Double> setpoints1 = List.of(0.0, 5.0, 10.8, 19.7, 32.0, 5.0);
  private List<Double> setpoints2 = List.of(0.0, 17.0, 30.0, 35.0, 4.5, 5.5);
  private List<Double> activeSetpoints = setpoints1; // Default to setpoints
  private TalonFX le = new TalonFX(13, "Drivetrain");
  private TalonFX re = new TalonFX(14, "Drivetrain");
  private PIDController pidup = new PIDController(0.05, 0, 0);
  private PIDController piddown = new PIDController(0.02, 0, 0);
  TalonFXConfiguration cfg = new TalonFXConfiguration();
  private final VoltageOut m_sysIdControl = new VoltageOut(0);
  private Slot0Configs slot0 = cfg.Slot0;
  MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  final MotionMagicVoltage m_request1 = new MotionMagicVoltage(0);

  private enum ElevatorState {
    MOVING, // using Motion Magic to drive to a setpoint
    HOLDING // using a WPILib PID controller to hold the position
  }

  private ElevatorState m_state = ElevatorState.HOLDING;

  // The target position (in sensor units) that you want to reach
  private double m_targetPosition = 0.0;

  // Tolerances for switching from MOVING to HOLDING
  private final double kPositionTolerance = 5; // adjust as needed
  private final double kVelocityTolerance = 0.1;

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(0.01), // Reduce dynamic voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              state -> SignalLogger.writeString("Elevator State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> le.setControl(m_sysIdControl.withOutput(volts)), null, this));
  private final SysIdRoutine m_sysIdRoutine2 =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(1), // Reduce dynamic voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> re.setControl(m_sysIdControl.withOutput(volts)), null, this));

  public elevatorpid() {
    slot0.kG = 0.1; // A gear ratio of 4:1 results in 0.25 output
    slot0.kS = 0.25;
    slot0.kV = 0.18; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 1.5; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0.1; // no output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        300; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 900; // Target jerk of 1600 rps/s/s (0.1 seconds)\

    le.getConfigurator().apply(cfg);
    re.getConfigurator().apply(cfg);

    // Follower followrequest = new Follower(le.getDeviceID(), true);

    SignalLogger.start();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("left", le.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("right", -re.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("voltage1", le.getMotorVoltage().getValueAsDouble());

    re.setControl(new Follower(le.getDeviceID(), true));
  }

  public void resetenc() {
    le.setPosition(0);
    re.setPosition(0);
  }

  public double getLeftPosition() {
    return le.getPosition().getValueAsDouble();
  }

  public void setMotionMagic(double position) {
    // Use Motion Magic with feedforward and FOC enabled.
    le.setControl(m_request.withPosition(position).withFeedForward(0.15).withEnableFOC(true));
    // If you want the follower to track, you can do the same for 're' if needed.
  }

  public void setMotionMagic1(int position) {
    // Use Motion Magic with feedforward and FOC enabled.
    le.setControl(
        m_request
            .withPosition(activeSetpoints.get(position))
            .withFeedForward(0.15)
            .withEnableFOC(true));
    // If you want the follower to track, you can do the same for 're' if needed.
  }

  public void initializePid(double position) {
    pidup.reset();
    pidup.setSetpoint(position);
  }

  public void setPidOutput(double position) {
    double speed = pidup.calculate(le.getPosition().getValueAsDouble());
    le.set(speed);
  }

  public void stop() {
    le.setControl(new VoltageOut(0));
    re.setControl(new VoltageOut(0));
  }

  public Command cmd(double position) {
    return new Command() {
      @Override
      public void initialize() {
        // Initialization code, such as resetting encoders or PID controllers
        // int kErrThreshold = 10; // how many sensor units until its close-enough
        // int kLoopsToSettle = 2; // how many loops sensor must be close-enough
        // int _withinThresholdLoops = 0;
      }

      @Override
      public void execute() {
        // check(position);
        re.setPosition(-re.getPosition().getValueAsDouble());

        le.setControl(m_request.withPosition(position).withFeedForward(0.15));
        re.setControl(m_request.withPosition(position).withFeedForward(0.15));
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command CTREpid(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.1;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed

      }

      @Override
      public void execute() {
        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        le.setControl(m_request.withPosition(targetPosition).withEnableFOC(true));
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public Command Motionmagictoggle(int value) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.25;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed

      }

      @Override
      public void execute() {

        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        le.setControl(
            m_request
                .withPosition(activeSetpoints.get(value))
                .withFeedForward(0.4)
                .withEnableFOC(true));
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - activeSetpoints.get(value));
        return kTolerance > error;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public Command Motionmagic(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.25;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed
        pidup.setSetpoint(targetPosition);
      }

      @Override
      public void execute() {

        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        le.setControl(
            m_request.withPosition(targetPosition).withFeedForward(0.4).withEnableFOC(true));
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
        return kTolerance > error;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public Command pidup(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.1;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed
        pidup.setSetpoint(targetPosition);
      }

      @Override
      public void execute() {
        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        double speed = pidup.calculate(le.getPosition().getValueAsDouble());
        le.set(speed);
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public Command piddown(double targetPosition) {
    return new Command() {
      // Define a tolerance (adjust as needed based on your sensor units)
      private final double kTolerance = 0.1;

      @Override
      public void initialize() {
        // Optionally reset any state or encoders if needed
        piddown.setSetpoint(targetPosition);
      }

      @Override
      public void execute() {
        // Command the leader motor using Motion Magic with feedforward.
        // (Since re is meant to follow le, remove direct control of re here.)

        double speed = piddown.calculate(le.getPosition().getValueAsDouble());
        le.set(speed);
      }

      @Override
      public boolean isFinished() {
        // End the command once the error is within tolerance.
        double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        // Once finished (or interrupted), stop the motors.
        le.setControl(new VoltageOut(0));
        // If you’re using a follower, you can let it follow automatically.
        // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
        re.setControl(new VoltageOut(0));
      }
    };
  }

  public Command runCurrentZeroing() {
    return this.run(
            () -> {
              // Run: Command the leader to drive downward at -1.0 V.
              le.setControl(m_request.withPosition(0));
            })
        .until(
            () -> {
              // Continue running until the current exceeds 40 A.
              // Adjust the method call based on your Phoenix 6 API (e.g., getStatorCurrent() or
              // getSupplyCurrent()).
              return le.getStatorCurrent().getValueAsDouble() > 40;
            })
        .andThen(
            new InstantCommand(
                () -> {
                  le.setControl(new VoltageOut(0));
                  re.setControl(new VoltageOut(0));
                }))
        // Wait for 0.5 seconds.
        .andThen(new WaitCommand(0.5))
        // After waiting, reset the encoders.
        .andThen(
            new InstantCommand(
                () -> {
                  le.setPosition(0.0);
                  re.setPosition(0.0);
                }))
        .beforeStarting(
            () -> {
              // Before starting, perform any one-time actions. For example, set a locking servo:
              // io.setLockServoRotation(0.2);
              // (Replace with your actual method if you have a servo to engage.)
              System.out.println("Starting currxent zeroing routine");
            });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public void togglesetpoint() {
    activeSetpoints = (activeSetpoints == setpoints1) ? setpoints2 : setpoints1;
  }
}
