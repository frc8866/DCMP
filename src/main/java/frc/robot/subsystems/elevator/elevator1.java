// package frc.robot.subsystems.elevator;

// import static edu.wpi.first.units.Units.Volts;

// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.*;
// import java.util.List;

// public class elevator1 extends SubsystemBase {
//   private List<Double> setpoints1 = List.of(1.0, 5.0, 10.0, 15.0, 4.0, 5.0);
//   private List<Double> setpoints2 = List.of(20.0, 25.0, 30.0, 35.0, 4.5, 5.5);
//   private List<Double> activeSetpoints = setpoints1; // Default to setpoints
//   private TalonFX le = new TalonFX(13, "Drivetrain");
//   private TalonFX re = new TalonFX(14, "Drivetrain");
//   TalonFXConfiguration cfg = new TalonFXConfiguration();
//   private final VoltageOut m_sysIdControl = new VoltageOut(0);
//   private Slot0Configs slot0 = cfg.Slot0;
//   MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
//   final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
//   final MotionMagicVoltage m_request1 = new MotionMagicVoltage(0);

//   private final SysIdRoutine m_sysIdRoutine =
//       new SysIdRoutine(
//           new SysIdRoutine.Config(
//               null, // Use default ramp rate (1 V/s)
//               Volts.of(1), // Reduce dynamic voltage to 4 to prevent brownout
//               null, // Use default timeout (10 s)
//               // Log state with Phoenix SignalLogger class
//               state -> SignalLogger.writeString("state", state.toString())),
//           new SysIdRoutine.Mechanism(
//               volts -> le.setControl(m_sysIdControl.withOutput(volts)), null, this));
//   private final SysIdRoutine m_sysIdRoutine2 =
//       new SysIdRoutine(
//           new SysIdRoutine.Config(
//               null, // Use default ramp rate (1 V/s)
//               Volts.of(1), // Reduce dynamic voltage to 4 to prevent brownout
//               null, // Use default timeout (10 s)
//               // Log state with Phoenix SignalLogger class
//               state -> SignalLogger.writeString("state", state.toString())),
//           new SysIdRoutine.Mechanism(
//               volts -> re.setControl(m_sysIdControl.withOutput(volts)), null, this));

//   public elevator1() {
//     slot0.kG = 0.4; // A gear ratio of 4:1 results in 0.25 output
//     slot0.kS = 0.25;
//     slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
//     slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
//     slot0.kP = 1; // A position error of 2.5 rotations results in 12 V output
//     slot0.kI = 0; // no output for integrated error
//     slot0.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

//     motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
//     motionMagicConfigs.MotionMagicAcceleration =
//         250; // Target acceleration of 160 rps/s (0.5 seconds)
//     motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)\

//     le.getConfigurator().apply(cfg);
//     re.getConfigurator().apply(cfg);

//     // Follower followrequest = new Follower(le.getDeviceID(), true);

//     SignalLogger.start();
//   }

//   @Override
//   public void periodic() {

//     SmartDashboard.putNumber("left", le.getPosition().getValueAsDouble());
//     SmartDashboard.putNumber("right", -re.getPosition().getValueAsDouble());

//     re.setControl(new Follower(le.getDeviceID(), true));
//   }

//   public void resetenc() {
//     le.setPosition(0);
//     re.setPosition(0);
//   }

//   public Command cmd(double position) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         // Initialization code, such as resetting encoders or PID controllers
//         // int kErrThreshold = 10; // how many sensor units until its close-enough
//         // int kLoopsToSettle = 2; // how many loops sensor must be close-enough
//         // int _withinThresholdLoops = 0;
//       }

//       @Override
//       public void execute() {
//         // check(position);
//         re.setPosition(-re.getPosition().getValueAsDouble());

//         le.setControl(m_request.withPosition(position).withFeedForward(0.15));
//         re.setControl(m_request.withPosition(position).withFeedForward(0.15));
//       }

//       @Override
//       public void end(boolean interrupted) {}

//       @Override
//       public boolean isFinished() {
//         return false; // Check if the setpoint is reached
//       }
//     };
//   }

//   public Command cmd1(double targetPosition) {
//     return new Command() {
//       private final double kTolerance = 0.1;

//       @Override
//       public void initialize() {
//         // Any necessary initialization
//       }

//       @Override
//       public void execute() {
//         double currentPos = le.getPosition().getValueAsDouble();
//         // If above a small threshold, command position control; otherwise, stop.
//         if (currentPos > 0.2) {
//           le.setControl(m_request.withPosition(targetPosition).withFeedForward(0.15));
//         } else {
//           le.setControl(new VoltageOut(0));
//           // It’s better not to mix setControl calls for re if it’s already following le.
//           // re.setControl(new VoltageOut(0));
//         }
//       }

//       @Override
//       public boolean isFinished() {
//         // End when the position is close to the target (0 in this case).
//         return Math.abs(le.getPosition().getValueAsDouble() - targetPosition) < kTolerance;
//       }

//       @Override
//       public void end(boolean interrupted) {
//         le.setControl(new VoltageOut(0));
//         re.setControl(new VoltageOut(0));
//       }
//     };
//   }

//   public Command Elevotorcmd(int index) {

//     return new Command() {
//       @Override
//       public void initialize() {
//         // Initialization code, such as resetting encoders or PID controllers

//       }

//       @Override
//       public void execute() {
//         le.setControl(m_request.withPosition(activeSetpoints.get(index)));

//         // Ele2.set(-speed); // Reverse motor direction if needed
//       }

//       @Override
//       public void end(boolean interrupted) {
//         le.set(0);
//       }

//       @Override
//       public boolean isFinished() {
//         return false;
//       }
//     };
//   }

//   public Command cmd2(double targetPosition) {
//     return new Command() {
//       // Define a tolerance (adjust as needed based on your sensor units)
//       private final double kTolerance = 0.1;

//       @Override
//       public void initialize() {
//         // Optionally reset any state or encoders if needed
//       }

//       @Override
//       public void execute() {
//         // Command the leader motor using Motion Magic with feedforward.
//         // (Since re is meant to follow le, remove direct control of re here.)
//         le.setControl(m_request.withPosition(targetPosition).withFeedForward(0.3));
//       }

//       @Override
//       public boolean isFinished() {
//         // End the command once the error is within tolerance.
//         double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
//         return false;
//       }

//       @Override
//       public void end(boolean interrupted) {
//         // Once finished (or interrupted), stop the motors.
//         le.setControl(new VoltageOut(0));
//         // If you’re using a follower, you can let it follow automatically.
//         // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
//         re.setControl(new VoltageOut(0));
//       }
//     };
//   }

//   public Command zeroElevator() {
//     return new Command() {
//       // Tune these constants as needed for your elevator.
//       private final double STALL_CURRENT_THRESHOLD = 40.0; // Amps (adjust based on your system)
//       private final double DOWNWARD_VOLTAGE = -1.0; // Volts (a gentle speed; adjust as needed)
//       private final int REQUIRED_LOOPS = 3; // Number of consecutive loops above threshold
//       private int loopsAboveThreshold = 0;

//       @Override
//       public void initialize() {
//         loopsAboveThreshold = 0;
//         // Optionally log that the zeroing routine has started.
//         System.out.println("Zeroing elevator: driving downward.");
//       }

//       @Override
//       public void execute() {
//         // Command the leader motor to drive downward slowly.
//         // (Assuming a VoltageOut control works for this purpose.)
//         le.setControl(new VoltageOut(DOWNWARD_VOLTAGE));

//         // If your follower (re) is meant to follow le automatically, you only need to configure
//         // that once
//         // (for example, in the constructor). If not, you could command it similarly.
//       }

//       @Override
//       public boolean isFinished() {
//         // Retrieve the current from the motor. (Make sure to use the appropriate method per
// Phoenix
//         // 6 docs.)
//         double current = le.getStatorCurrent().getValueAsDouble();
//         // Uncomment and use the below line if your API uses a different current method.
//         // double current = le.getSupplyCurrent().getValueAsDouble();

//         // If the measured current exceeds the threshold, increment our counter.
//         if (current >= STALL_CURRENT_THRESHOLD) {
//           loopsAboveThreshold++;
//         } else {
//           // Reset the counter if the current drops below the threshold.
//           loopsAboveThreshold = 0;
//         }
//         // When we've seen the threshold exceeded for the required number of loops, finish the
//         // command.
//         return loopsAboveThreshold >= REQUIRED_LOOPS;
//       }

//       @Override
//       public void end(boolean interrupted) {
//         // Stop the motors.
//         le.setControl(new VoltageOut(0));
//         re.setControl(new VoltageOut(0));

//         // Reset encoder positions. This assumes that reaching the stall means you are at the
//         // bottom.
//         le.setPosition(0);
//         re.setPosition(0);

//         System.out.println("Elevator zeroed.");
//       }
//     };
//   }

//   public Command runCurrentZeroing() {
//     return this.run(
//             () -> {
//               // Run: Command the leader to drive downward at -1.0 V.
//               le.setControl(m_request.withPosition(0));
//             })
//         .until(
//             () -> {
//               // Continue running until the current exceeds 40 A.
//               // Adjust the method call based on your Phoenix 6 API (e.g., getStatorCurrent() or
//               // getSupplyCurrent()).
//               return le.getStatorCurrent().getValueAsDouble() > 40;
//             })
//         .andThen(
//             new InstantCommand(
//                 () -> {
//                   le.setControl(new VoltageOut(0));
//                   re.setControl(new VoltageOut(0));
//                 }))
//         // Wait for 0.5 seconds.
//         .andThen(new WaitCommand(0.5))
//         // After waiting, reset the encoders.
//         .andThen(
//             new InstantCommand(
//                 () -> {
//                   le.setPosition(0.0);
//                   re.setPosition(0.0);
//                 }))
//         .beforeStarting(
//             () -> {
//               // Before starting, perform any one-time actions. For example, set a locking servo:
//               // io.setLockServoRotation(0.2);
//               // (Replace with your actual method if you have a servo to engage.)
//               System.out.println("Starting currxent zeroing routine");
//             });
//   }

//   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//     return new ParallelCommandGroup(
//         m_sysIdRoutine.quasistatic(direction), m_sysIdRoutine2.quasistatic(direction));
//   }

//   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//     return new ParallelCommandGroup(
//         m_sysIdRoutine.quasistatic(direction), m_sysIdRoutine2.quasistatic(direction));
//   }

//   public void togglesetpoint() {
//     activeSetpoints = (activeSetpoints == setpoints1) ? setpoints2 : setpoints1;
//   }
// }
