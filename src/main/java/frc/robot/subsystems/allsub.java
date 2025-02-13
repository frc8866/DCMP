// package frc.robot.subsystems;

// import au.grapplerobotics.ConfigurationFailedException;
// import au.grapplerobotics.LaserCan;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import java.util.List;

// public class allsub extends SubsystemBase {
//   private TalonFX lshoot = new TalonFX(17);
//   private TalonFX rshoot = new TalonFX(18);
//   private LaserCan lc = new LaserCan(0);
//   private TalonFX hopper = new TalonFX(30);
//   private int distance;
//   private Timer time3 = new Timer();
//   private List<Double> setpoints1 = List.of(0.0, 5.0, 10.8, 19.7, 32.0, 5.0);
//   private List<Double> setpoints2 = List.of(0.0, 17.0, 30.0, 35.0, 4.5, 5.5);
//   private List<Double> activeSetpoints = setpoints1; // Default to setpoints
//   private TalonFX le = new TalonFX(13, "Drivetrain");
//   private TalonFX re = new TalonFX(14, "Drivetrain");
//   private PIDController pidup = new PIDController(0.05, 0, 0);
//   private PIDController piddown = new PIDController(0.02, 0, 0);
//   TalonFXConfiguration cfg = new TalonFXConfiguration();
//   private final VoltageOut m_sysIdControl = new VoltageOut(0);
//   private Slot0Configs slot0 = cfg.Slot0;
//   MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
//   final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
//   private TalonFX algaepiv = new TalonFX(15);
//   private TalonFX algaeshooter = new TalonFX(16);
//   private PIDController pid = new PIDController(0.07, 0, 0);

//   private enum ElevatorState {
//     MOVING, // using Motion Magic to drive to a setpoint
//     HOLDING // using a WPILib PID controller to hold the position
//   }

//   public static enum RobotStateg {
//     IDLE, // Robot is not doing anything
//     MOVING, // Robot is driving
//     INTAKING, // Robot is picking up a game piece
//     SHOOTING, // Robot is shooting a game piece
//     CLIMBING,
//     ALGEA; // Robot is climbing
//   }

//   private RobotStateg m_robotState = RobotStateg.IDLE;

//   private ElevatorState m_state = ElevatorState.HOLDING;
//   // The target position (in sensor units) that you want to reach
//   private double m_targetPosition = 0.0;

//   // Tolerances for switching from MOVING to HOLDING
//   private final double kPositionTolerance = 5; // adjust as needed
//   private final double kVelocityTolerance = 0.1;

//   public allsub() {
//     slot0.kG = 0.1; // A gear ratio of 4:1 results in 0.25 output
//     slot0.kS = 0.25;
//     slot0.kV = 0.18; // A velocity target of 1 rps results in 0.12 V output
//     slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
//     slot0.kP = 1.5; // A position error of 2.5 rotations results in 12 V output
//     slot0.kI = 0.1; // no output for integrated error
//     slot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

//     motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
//     motionMagicConfigs.MotionMagicAcceleration =
//         300; // Target acceleration of 160 rps/s (0.5 seconds)
//     motionMagicConfigs.MotionMagicJerk = 900; // Target jerk of 1600 rps/s/s (0.1 seconds)\
//     lshoot.setNeutralMode(NeutralModeValue.Brake);
//     rshoot.setNeutralMode(NeutralModeValue.Brake);

//     le.getConfigurator().apply(cfg);
//     re.getConfigurator().apply(cfg);
//     // Initialization code, such as resetting encoders or PID controllers
//     algaepiv.setNeutralMode(NeutralModeValue.Brake);
//     le.setNeutralMode(NeutralModeValue.Brake);
//     re.setNeutralMode(NeutralModeValue.Brake);
//     try {
//       lc.setRangingMode(LaserCan.RangingMode.SHORT);
//       lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
//       lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
//     } catch (ConfigurationFailedException e) {
//       System.out.println("Configuration failed! " + e);
//     }
//   }

//   @Override
//   public void periodic() {

//     LaserCan.Measurement measurement = lc.getMeasurement();
//     if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
// {
//       SmartDashboard.putNumber("distance", measurement.distance_mm);
//       SmartDashboard.putBoolean("Beamborken", check());
//       distance = measurement.distance_mm;

//       // This method will be called once per scheduler run
//       SmartDashboard.putNumber("algaepiv", algaepiv.getPosition().getValueAsDouble());
//       SmartDashboard.putNumber("Algee", algaepiv.getMotorVoltage().getValueAsDouble());
//       SmartDashboard.putNumber("Velocity", algaeshooter.getVelocity().getValueAsDouble());
//       SmartDashboard.putNumber("voltage", algaepiv.getMotorVoltage().getValueAsDouble());
//       re.setControl(new Follower(le.getDeviceID(), true));
//     }
//   }

//   private boolean check() {
//     return distance > 75;
//   }

//   public void resetenc() {
//     le.setPosition(0);
//     re.setPosition(0);
//   }

//   public double getLeftPosition() {
//     return le.getPosition().getValueAsDouble();
//   }

//   public void setMotionMagic(double position) {
//     // Use Motion Magic with feedforward and FOC enabled.
//     le.setControl(m_request.withPosition(position).withFeedForward(0.15).withEnableFOC(true));
//     // If you want the follower to track, you can do the same for 're' if needed.
//   }

//   public void setMotionMagic1(int position) {
//     // Use Motion Magic with feedforward and FOC enabled.
//     le.setControl(
//         m_request
//             .withPosition(activeSetpoints.get(position))
//             .withFeedForward(0.15)
//             .withEnableFOC(true));
//     // If you want the follower to track, you can do the same for 're' if needed.
//   }

//   public void initializePid(double position) {
//     pidup.reset();
//     pidup.setSetpoint(position);
//   }

//   public void setPidOutput(double position) {
//     double speed = pidup.calculate(le.getPosition().getValueAsDouble());
//     le.set(speed);
//   }

//   public void stop() {
//     le.setControl(new VoltageOut(0));
//     re.setControl(new VoltageOut(0));
//   }

//   //   public Command cmd(double position) {
//   //     return new Command() {
//   //       @Override
//   //       public void initialize() {
//   //         // Initialization code, such as resetting encoders or PID controllers
//   //         // int kErrThreshold = 10; // how many sensor units until its close-enough
//   //         // int kLoopsToSettle = 2; // how many loops sensor must be close-enough
//   //         // int _withinThresholdLoops = 0;
//   //       }

//   //       @Override
//   //       public void execute() {
//   //         // check(position);
//   //         re.setPosition(-re.getPosition().getValueAsDouble());

//   //         le.setControl(m_request.withPosition(position).withFeedForward(0.15));
//   //         re.setControl(m_request.withPosition(position).withFeedForward(0.15));
//   //       }

//   //       @Override
//   //       public void end(boolean interrupted) {}

//   //       @Override
//   //       public boolean isFinished() {
//   //         return false; // Check if the setpoint is reached
//   //       }
//   //     };
//   //   }

//   public Command CTREpid(double targetPosition) {
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

//         le.setControl(m_request.withPosition(targetPosition).withEnableFOC(true));
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

//   public Command Motionmagictoggle(int value) {
//     return new Command() {
//       // Define a tolerance (adjust as needed based on your sensor units)
//       private final double kTolerance = 0.25;

//       @Override
//       public void initialize() {
//         // Optionally reset any state or encoders if needed

//       }

//       @Override
//       public void execute() {

//         // Command the leader motor using Motion Magic with feedforward.
//         // (Since re is meant to follow le, remove direct control of re here.)

//         le.setControl(
//             m_request
//                 .withPosition(activeSetpoints.get(value))
//                 .withFeedForward(0.4)
//                 .withEnableFOC(true));
//       }

//       @Override
//       public boolean isFinished() {
//         // End the command once the error is within tolerance.
//         double error = Math.abs(le.getPosition().getValueAsDouble() -
// activeSetpoints.get(value));
//         return kTolerance > error;
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

//   public Command Motionmagic(double targetPosition) {
//     return new Command() {
//       // Define a tolerance (adjust as needed based on your sensor units)
//       private final double kTolerance = 0.25;

//       @Override
//       public void initialize() {
//         // Optionally reset any state or encoders if needed
//         pidup.setSetpoint(targetPosition);
//       }

//       @Override
//       public void execute() {

//         // Command the leader motor using Motion Magic with feedforward.
//         // (Since re is meant to follow le, remove direct control of re here.)

//         le.setControl(
//             m_request.withPosition(targetPosition).withFeedForward(0.4).withEnableFOC(true));
//       }

//       @Override
//       public boolean isFinished() {
//         // End the command once the error is within tolerance.
//         double error = Math.abs(le.getPosition().getValueAsDouble() - targetPosition);
//         return kTolerance > error;
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

//   public Command pidup(double targetPosition) {
//     return new Command() {
//       // Define a tolerance (adjust as needed based on your sensor units)
//       private final double kTolerance = 0.1;

//       @Override
//       public void initialize() {
//         // Optionally reset any state or encoders if needed
//         pidup.setSetpoint(targetPosition);
//       }

//       @Override
//       public void execute() {
//         // Command the leader motor using Motion Magic with feedforward.
//         // (Since re is meant to follow le, remove direct control of re here.)

//         double speed = pidup.calculate(le.getPosition().getValueAsDouble());
//         le.set(speed);
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

//   public Command piddown(double targetPosition) {
//     return new Command() {
//       // Define a tolerance (adjust as needed based on your sensor units)
//       private final double kTolerance = 0.1;

//       @Override
//       public void initialize() {
//         // Optionally reset any state or encoders if needed
//         piddown.setSetpoint(targetPosition);
//       }

//       @Override
//       public void execute() {
//         // Command the leader motor using Motion Magic with feedforward.
//         // (Since re is meant to follow le, remove direct control of re here.)

//         double speed = piddown.calculate(le.getPosition().getValueAsDouble());
//         le.set(speed);
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

//   public void togglesetpoint() {
//     if (activeSetpoints == setpoints1) {
//       activeSetpoints = setpoints2;
//       m_robotState = RobotStateg.ALGEA;
//     } else {
//       activeSetpoints = setpoints1;
//       m_robotState = RobotStateg.IDLE; // or any other "normal" state you prefer
//     }
//   }

//   public RobotStateg getRobotState() {
//     return m_robotState;
//   }

//   public boolean isAlgeaMode() {
//     return m_robotState == RobotStateg.ALGEA;
//   }

//   public Command cmd(double speed) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         // Initialization code, such as resetting encoders or PID controllers
//       }

//       @Override
//       public void execute() {

//         lshoot.set(speed / 100);
//         rshoot.set(-speed / 100);
//       }

//       @Override
//       public void end(boolean interrupted) {
//         lshoot.set(0);
//         rshoot.set(0);
//       }

//       @Override
//       public boolean isFinished() {
//         return false; // Check if the setpoint is reached
//       }
//     };
//   }

//   public Command both(double sspeed, double hspeed) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         // Initialization code, such as resetting encoders or PID controllers
//       }

//       @Override
//       public void execute() {

//         if (distance > 75) {
//           lshoot.set(sspeed);
//           rshoot.set(-sspeed);
//           hopper.set(hspeed);

//         } else {
//           lshoot.set(0);
//           rshoot.set(0);
//           hopper.set(0);
//         }
//       }

//       @Override
//       public void end(boolean interrupted) {
//         hopper.set(0);
//         lshoot.set(0);
//         rshoot.set(0);
//       }

//       @Override
//       public boolean isFinished() {
//         return distance < 75; // 39 is a setpoint number i need to find the acc number
//       }
//     };
//   }

//   public Command hopper(double speed) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         // Initialization code, such as resetting encoders or PID controllers
//       }

//       @Override
//       public void execute() {

//         hopper.set(speed / 100);
//       }

//       @Override
//       public void end(boolean interrupted) {
//         hopper.set(0);
//       }

//       @Override
//       public boolean isFinished() {
//         return false; // Check if the setpoint is reached
//       }
//     };
//   }

//   public Command time1(double time, double speed, double speed2) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         // Initialization code, such as resetting encoders or PID controllers
//         time3.start();
//       }

//       @Override
//       public void execute() {

//         if (time3.get() < 0.3) {
//           hopper.set(speed);
//           lshoot.set(speed2);
//           rshoot.set(-speed2);
//         } else {
//           lshoot.set(0);
//           rshoot.set(0);
//         }
//       }

//       @Override
//       public void end(boolean interrupted) {
//         hopper.set(0);
//         time3.stop();
//         time3.reset();
//         lshoot.set(0);
//         rshoot.set(0);
//       }

//       @Override
//       public boolean isFinished() {
//         return time3.get() > time; // Check if the setpoint is reached
//       }
//     };
//   }

//   public Command cmd3(double speed, double speed2) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         // Initialization code, such as resetting encoders or PID controllers
//       }

//       @Override
//       public void execute() {

//         lshoot.set(speed / 100);
//         rshoot.set(-speed2 / 100);
//       }

//       @Override
//       public void end(boolean interrupted) {
//         lshoot.set(0);
//         rshoot.set(0);
//       }

//       @Override
//       public boolean isFinished() {
//         return false; // Check if the setpoint is reached
//       }
//     };
//   }

//   public void resetalgea() {
//     algaepiv.setPosition(0);
//   }

//   public double velocity() {
//     return algaeshooter.getVelocity().getValueAsDouble();
//   }

//   public void setShooter(double speed) {
//     algaeshooter.set(speed);
//   }

//   public void speed(double position) {
//     pid.setSetpoint(position);

//     double speed = pid.calculate(algaepiv.getPosition().getValueAsDouble());
//     algaepiv.set(speed);
//   }

//   public Command algeacmd(double position, double speed) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         pid.setSetpoint(position);
//         // Initialization code, such as resetting encoders or PID controllers
//       }

//       @Override
//       public void execute() {
//         algaeshooter.set(speed);

//         double speed1 = pid.calculate(algaepiv.getPosition().getValueAsDouble());
//         algaepiv.set(speed1);
//       }

//       @Override
//       public void end(boolean interrupted) {
//         algaeshooter.set(0);
//       }

//       @Override
//       public boolean isFinished() {
//         return false; // Check if the setpoint is reached; // Check if the setpoint is reached
//       }
//     };
//   }

//   public Command position(double position) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         pid.setSetpoint(position);
//         // Initialization code, such as resetting encoders or PID controllers
//       }

//       @Override
//       public void execute() {

//         double speed1 = pid.calculate(algaepiv.getPosition().getValueAsDouble());
//         algaepiv.set(speed1);
//       }

//       @Override
//       public void end(boolean interrupted) {}

//       @Override
//       public boolean isFinished() {
//         return false; // Check if the setpoint is reached; // Check if the setpoint is reached
//       }
//     };
//   }

//   public Command ion(double speed) {
//     return new Command() {
//       @Override
//       public void initialize() {
//         // Initialization code, such as resetting encoders or PID controllers
//       }

//       @Override
//       public void execute() {
//         algaepiv.set(speed);
//       }

//       @Override
//       public void end(boolean interrupted) {
//         algaeshooter.set(0);
//       }

//       @Override
//       public boolean isFinished() {
//         return false; // Check if the setpoint is reached; // Check if the setpoint is reached
//       }
//     };
//   }
// }
