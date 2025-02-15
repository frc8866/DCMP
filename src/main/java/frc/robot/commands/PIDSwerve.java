package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Pose;

import frc.robot.subsystems.drive.Drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Elevatorcmd;
import frc.robot.commands.HopperTriggeredShooterAndRumbleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOCTRE;
import frc.robot.subsystems.arm.ArmIOSIM;
import frc.robot.subsystems.arm.algee;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.requests.ProfiledFieldCentricFacingAngle;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
import frc.robot.subsystems.elevator.Ballintake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.elevator.elevatorpid;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSIM;
import frc.robot.subsystems.flywheel.shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PIDSwerve extends Command {
    private final Drive s_Swerve;

    private final PIDController xPID = new PIDController(0.05, 0, 0);
    private final PIDController yPID = new PIDController(0.05, 0, 0);
    private final double positionTolerance = 1.0; // inches
    // private final double maxSpeed = Constants.Swerve.maxSpeed / 5.0;
    private final double positionKS = 0.02;
    private final double positionIZone = 4.0;

    private final PIDController rotationPID = new PIDController(0.003, 0, 0);
    private final double rotationTolerance = 1.5; // degrees

    private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;


    private final SwerveRequest.RobotCentric baseRequest =
          new SwerveRequest.RobotCentric()
              .withDeadband(MaxSpeed.times(0.1))
              .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage);




    // private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;

    public PIDSwerve(Drive s_Swerve,  Pose2d targetPose) {
        super();

        this.s_Swerve = s_Swerve;
      ;
        addRequirements(s_Swerve);

        xPID.setIZone(positionIZone); // Only use Integral term within this range
        xPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        xPID.setSetpoint(Units.metersToInches(targetPose.getX()));
        xPID.setTolerance(positionTolerance);
    

        yPID.setIZone(positionIZone); // Only use Integral term within this range
        yPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        yPID.setSetpoint(Units.metersToInches(targetPose.getY())); // TODO Set derivative, too
        yPID.setTolerance(positionTolerance);


        rotationPID.enableContinuousInput(-180.0, 180.0);
        rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
        rotationPID.setIntegratorRange(-Pose.rotationKS * 2, Pose.rotationKS * 2);
        rotationPID.setSetpoint(targetPose.getRotation().getDegrees());
        rotationPID.setTolerance(rotationTolerance); // TODO Set derivative, too

        
    }

    @Override
    public void initialize() {
        super.initialize();

        xPID.reset();
        yPID.reset();
        rotationPID.reset();
    }

    @Override
    public void execute() {
        Pose2d pose = s_Swerve.getPose();
        Translation2d position = pose.getTranslation();
        Rotation2d rotation = pose.getRotation();

        /* TODO Consider a potential need to rotate most of the way first, then translate */

        double xCorrection = xPID.calculate(Units.metersToInches(position.getX()));
        double xFeedForward = positionKS * Math.signum(xCorrection);
        double xVal = MathUtil.clamp(xCorrection + xFeedForward, -1.0, 1.0);
    

        double yCorrection = yPID.calculate(Units.metersToInches(position.getY()));
        double yFeedForward = positionKS * Math.signum(yCorrection);
        double yVal = MathUtil.clamp(yCorrection + yFeedForward, -1.0, 1.0);
    

        double correction = rotationPID.calculate(rotation.getDegrees());
        double feedForward = Pose.rotationKS * Math.signum(correction);
        double rotationVal = MathUtil.clamp(correction + feedForward, -1.0, 1.0);


        SwerveRequest.RobotCentric request = baseRequest
            .withVelocityX(MaxSpeed.times(xVal))
            .withVelocityY(MaxSpeed.times(yVal))
            .withRotationalRate(Constants.MaxAngularRate.times(rotationVal));

            s_Swerve.applyRequest(() -> request);
        

      
    }

    @Override
    public boolean isFinished() {
        return xPID.atSetpoint() && yPID.atSetpoint() && rotationPID.atSetpoint();
    }
}