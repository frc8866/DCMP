// package frc.robot.subsystems.elevator;

// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.*;

// public class ele extends SubsystemBase {

//   //////////////////////////////////////////////////////////////////////////////
//   // State Machine for switching between Motion Magic (to move) and PID hold //
//   //////////////////////////////////////////////////////////////////////////////
//   private enum ElevatorState {
//     MOVING, // using Motion Magic to drive to a setpoint
//     HOLDING // using a WPILib PID controller to hold the position
//   }

//   private PIDController piddown = new PIDController(0.02, 0, 0);

//   // Start in HOLDING state (assume you are already at your desired position)
//   private ElevatorState m_state = ElevatorState.HOLDING;

//   // The target position (in sensor units) that you want to reach
//   private double m_targetPosition = 0.0;

//   // Tolerances for switching from MOVING to HOLDING
//   private final double kPositionTolerance = 0.25; // adjust as needed
//   private final double kVelocityTolerance = 0.1; // if you can obtain a velocity value

//   //////////////////////////////////////////////////////////////////////////////
//   // CTRE Phoenix6 Motor Setup (Motion Magic configuration)                   //
//   //////////////////////////////////////////////////////////////////////////////
//   private final TalonFX m_leader = new TalonFX(13, "Drivetrain");
//   private final TalonFX m_follower = new TalonFX(14, "Drivetrain");

//   TalonFXConfiguration m_config = new TalonFXConfiguration();
//   private final Slot0Configs m_slot0 = m_config.Slot0;
//   MotionMagicConfigs m_motionMagicConfigs = m_config.MotionMagic;
//   // Create a MotionMagicVoltage request (start at 0)
//   final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0);

//   //////////////////////////////////////////////////////////////////////////////
//   // WPILib PID Controller for holding the setpoint (you can tune these)        //
//   //////////////////////////////////////////////////////////////////////////////
//   private final PIDController m_holdPID = new PIDController(.01, 0.0, 0.0);

//   public ele() {
//     ///////////////////////////////
//     // Configure CTRE Parameters //
//     ///////////////////////////////
//     m_slot0.kG = 0.4;
//     m_slot0.kS = 0.25;
//     m_slot0.kV = 0.3;
//     m_slot0.kA = 0.01;
//     m_slot0.kP = 2.5;
//     m_slot0.kI = 0.0;
//     m_slot0.kD = 0.0;

//     // Configure Motion Magic parameters:
//     m_motionMagicConfigs.MotionMagicCruiseVelocity = 100; // target cruise velocity
//     m_motionMagicConfigs.MotionMagicAcceleration = 250; // target acceleration
//     m_motionMagicConfigs.MotionMagicJerk = 800; // target jerk

//     // Apply configuration to both the leader and follower
//     m_leader.getConfigurator().apply(m_config);
//     m_follower.getConfigurator().apply(m_config);

//     ////////////////////////////
//     // Set up Follower Motor  //
//     ////////////////////////////
//     // Make the follower mirror the leader.
//     // (In periodic() we issue a follower command so that if the leader’s control mode changes,
//     // the follower automatically tracks it.)

//     ////////////////////////////
//     // Start Logging (Optional) //
//     ////////////////////////////
//     SignalLogger.start();

//     ////////////////////////////////
//     // Initialize the State & PID //
//     ////////////////////////////////
//     // Assume that on startup the elevator is at its current (desired) position.
//     m_targetPosition = m_leader.getPosition().getValueAsDouble();
//     m_holdPID.setSetpoint(m_targetPosition);
//     m_holdPID.reset();
//     m_state = ElevatorState.HOLDING;
//   }

//   @Override
//   public void periodic() {
//     // Read the current position from the leader (assuming sensor units are appropriate)
//     double currentPosition = m_leader.getPosition().getValueAsDouble();
//     SmartDashboard.putNumber("Elevator Position", m_leader.getPosition().getValueAsDouble());
//     SmartDashboard.putString("Elevator State", m_state.toString());
//     SmartDashboard.putBoolean("check", getState() == m_state.HOLDING);

//     // Update the follower so it follows the leader
//     m_follower.setControl(new Follower(m_leader.getDeviceID(), true));

//     //////////////////////////////////////////////////////////////////////////////
//     // State Machine: Decide whether to use Motion Magic (MOVING) or PID hold (HOLDING) //
//     //////////////////////////////////////////////////////////////////////////////

//   }

//   /**
//    * Call this method to command the elevator to move to a new setpoint. It w ill use Motion
// Magic
//    * to move to the target and then automatically switch to PID hold once close enough.
//    *
//    * @param newTarget The desired position in sensor units.
//    */
//   public void setTargetPosition(double newTarget) {
//     m_targetPosition = newTarget;
//     // Switch state to MOVING so that Motion Magic is used to drive to the target.
//     m_state = ElevatorState.MOVING;
//   }

//   /** Optionally, you might want a method to immediately hold the current position. */
//   public void holdCurrentPosition() {
//     m_targetPosition = m_leader.getPosition().getValueAsDouble();
//     m_holdPID.setSetpoint(m_targetPosition);
//     m_holdPID.reset();
//     m_state = ElevatorState.HOLDING;
//   }

//   public ElevatorState getState() {
//     return m_state;
//   }

//   public Command elevatorCmd(final double setpoint) {
//     return new Command() {

//       @Override
//       public void initialize() {
//         // Set the new target position. This sets the state to MOVING.
//         setTargetPosition(setpoint);
//       }

//       @Override
//       public void execute() {
//         double currentPosition = m_leader.getPosition().getValueAsDouble();

//         if (m_state == ElevatorState.MOVING) {
//           // Command Motion Magic with a feedforward term (adjust feedforward as needed)
//           m_leader.setControl(
//               m_motionMagicRequest.withPosition(m_targetPosition).withEnableFOC(true));

//           // Optionally, if you can read velocity you could include a check here.
//           // For now, we simply switch to HOLDING once we’re within the position tolerance.
//           if (Math.abs(currentPosition - m_targetPosition) < kPositionTolerance) {
//             // Switch to hold mode:
//             m_state = ElevatorState.HOLDING;
//             m_holdPID.setSetpoint(m_targetPosition);
//             m_holdPID.reset();
//           }
//         } else if (m_state == ElevatorState.HOLDING) {
//           // Use WPILib PID to hold the current setpoint.
//           // The PID output will be used to command a voltage to the leader motor.
//           double pidOutput = m_holdPID.calculate(currentPosition);
//           m_leader.set(pidOutput);
//         }
//         // No additional execution needed since periodic() handles the control.
//       }

//       @Override
//       public boolean isFinished() {
//         // Finish the command when the elevator has switched to HOLDING mode.
//         return getState() == ElevatorState.HOLDING;
//       }

//       @Override
//       public void end(boolean interrupted) {
//         // Once finished, ensure we hold the current position.
//         holdCurrentPosition();
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

//         double speed = piddown.calculate(m_leader.getPosition().getValueAsDouble());
//         m_leader.set(speed);
//       }

//       @Override
//       public boolean isFinished() {
//         // End the command once the error is within tolerance.
//         double error = Math.abs(m_leader.getPosition().getValueAsDouble() - targetPosition);
//         return false;
//       }

//       @Override
//       public void end(boolean interrupted) {
//         // Once finished (or interrupted), stop the motors.
//         m_leader.setControl(new VoltageOut(0));
//         // If you’re using a follower, you can let it follow automatically.
//         // Alternatively, if you’re controlling 're' elsewhere, ensure it’s set to zero.
//         m_leader.setControl(new VoltageOut(0));
//       }
//     };
//   }
// }
