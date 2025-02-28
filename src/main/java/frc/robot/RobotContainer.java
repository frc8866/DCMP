package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutonElevatorcmd;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Elevatorcmd;
import frc.robot.commands.barge;
import frc.robot.commands.l3algae;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOCTRE;
import frc.robot.subsystems.arm.ArmIOSIM;
import frc.robot.subsystems.arm.algee;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.elevator.elevatorsub;
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

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final TunableController joystick =
      new TunableController(0).withControllerType(TunableControllerType.CUBIC);

  private final TunableController joystick2 =
      new TunableController(1).withControllerType(TunableControllerType.QUADRATIC);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final Flywheel flywheel;
  private final Elevator elevator;
  private final Arm arm;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
  private shooter shoot = new shooter();
  private elevatorsub elevator1 = new elevatorsub();
  private algee algea = new algee();

  public RobotContainer() {
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);

        new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVision(
                "FrontRight",
                new Transform3d(
                    new Translation3d(0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(5))),
                drivetrain::getVisionParameters),
            new VisionIOLimelight("limelight-fr", drivetrain::getVisionParameters),
            new VisionIOLimelight("limelight-bl", drivetrain::getVisionParameters),
            new VisionIOLimelight("limelight-br", drivetrain::getVisionParameters));

        // flywheel = new Flywheel(new FlywheelIOCTRE()); // Disabled to prevent robot movement if
        // deployed to a real robot
        flywheel = new Flywheel(new FlywheelIO() {});
        // elevator = new Elevator(new ElevatorIOCTRE()); // Disabled to prevent robot movement if
        // deployed to a real robot
        elevator = new Elevator(new ElevatorIO() {});
        // arm = new Arm(new ArmIOCTRE()); // Disabled to prevent robot movement if deployed to a
        // real robot
        arm = new Arm(new ArmIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);

        new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVisionSIM(
                "Front Camera",
                new Transform3d(
                    new Translation3d(0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(0))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Back Camera",
                new Transform3d(
                    new Translation3d(-0.2, 0.0, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(180))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Left Camera",
                new Transform3d(
                    new Translation3d(0.0, 0.2, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(90))),
                drivetrain::getVisionParameters),
            new VisionIOPhotonVisionSIM(
                "Right Camera",
                new Transform3d(
                    new Translation3d(0.0, -0.2, 0.8),
                    new Rotation3d(0, Math.toRadians(20), Math.toRadians(-90))),
                drivetrain::getVisionParameters));

        flywheel = new Flywheel(new FlywheelIOSIM());
        elevator = new Elevator(new ElevatorIOSIM());
        arm = new Arm(new ArmIOSIM());
        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = new Drive(new DriveIO() {});

        new Vision(
            drivetrain::addVisionData,
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {},
            new VisionIO() {});

        flywheel = new Flywheel(new FlywheelIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIOCTRE() {});
        break;
    }

    NamedCommands.registerCommand("elevatoru", new AutonElevatorcmd(elevator1, 3));
    NamedCommands.registerCommand("elevatord", new AutonElevatorcmd(elevator1, 0));
    // NamedCommands.registerCommand("shoot", shoot.AutonShoot(.07, 0));
    // NamedCommands.registerCommand(
    //     "intake", new SequentialCommandGroup(shoot.both(0.07, 0.25), shoot.autonwait(0.2, 0.1)));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drivetrain));
    configureBindings();
  }

  private void configureBindings() {

    // joystick.b().whileTrue(elevator1.cmdf(-26.6033203125)).whileFalse(elevator1.Flipydo(0));

    // check for FLIPPY DO POSITION

    Command Positionl2 =
        new SequentialCommandGroup(
            new ParallelCommandGroup(

                // setpoint
                new l3algae(algea, 0.7, 5, elevator1, -11.679, 0)));
    Command Positionl3 =
        // new SequentialCommandGroup(new l2algae(algea, 0.8, 5, elevator1, -18.31416015625));
        new SequentialCommandGroup(new l3algae(algea, 0.5, 5, elevator1, -14.83251953125, 5.32));

    // idk what to put for the setpoint as of now we havent tested it yet

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -joystick
                                .customLeft()
                                .getY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -joystick.customLeft().getX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -joystick
                                .customRight()
                                .getX())))); // Drive counterclockwise with negative X (left)

    // joystick  nidwj f

    joystick.b().onTrue(elevator1.runOnce(() -> elevator1.resetenc()));

    joystick
        .leftTrigger(0.2)
        .whileTrue(
            new ParallelCommandGroup(shoot.cmd(0.5), elevator1.runOnce(() -> elevator1.resetenc())))
        .whileFalse(shoot.cmd(0.05));

    // joystick.leftTrigger(0.2)
    // .whileTrue(new ConditionalCommand(new IntakeWithRumble(shoot, joystick, 0.3), Positionl3,
    // null))

    joystick
        .rightTrigger(0.2)
        .whileTrue(
            new ConditionalCommand(
                shoot.cmd(-0.2),
                algea.algeacmd(-0.5),
                () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
        .whileFalse(new ParallelCommandGroup(shoot.cmd(0.05)));

    joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.resetgyro()));
    joystick
        .back()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(MaxSpeed.times(-joystick.customLeft().getY()).times(0.5))
                        .withVelocityY(MaxSpeed.times(-joystick.customLeft().getX()).times(0.5))
                        .withRotationalRate(
                            Constants.MaxAngularRate.times(-joystick.customRight().getX())
                                .times(0.5))));

    joystick
        .a()
        .whileTrue(new Elevatorcmd(elevator1, 1, true))
        .whileFalse(
            new SequentialCommandGroup(
                elevator1.Motionmagictoggle(0),
                new ParallelCommandGroup(new Elevatorcmd(elevator1, 0, false))));

    joystick
        .pov(0)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));

    joystick
        .pov(180)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    joystick
        .pov(90)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));

    joystick
        .pov(270)
        .whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));

    joystick
        .rightStick()
        .whileTrue(
            new ConditionalCommand(
                new Elevatorcmd(elevator1, 2, true),
                Positionl2, // algae
                () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
        .whileFalse(
            new SequentialCommandGroup(
                elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));
    // joystick
    //     .rightStick()
    //     .whileTrue(new Elevatorcmd(elevator1, 2, true))
    //     .whileFalse(
    //         new SequentialCommandGroup(
    //             elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));
    ;
    joystick
        .leftStick()
        .whileTrue(
            new ConditionalCommand(
                new Elevatorcmd(elevator1, 3, true),
                Positionl3,
                () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
        .whileFalse(
            new SequentialCommandGroup(
                elevator1.Motionmagictoggle(0),
                new ParallelCommandGroup(new Elevatorcmd(elevator1, 0, false))));

    joystick
        .back()
        .whileTrue(
            new ConditionalCommand(
                new Elevatorcmd(elevator1, 4, true),
                new barge(elevator1, 26.841796875, true),
                () -> Constants.getRobotState() != Constants.RobotState.ALGEA))
        .whileFalse(
            new SequentialCommandGroup(
                elevator1.Motionmagictoggle(0), new Elevatorcmd(elevator1, 0, false)));
    joystick.rightBumper().whileTrue(algea.algeacmd(0.5)).whileFalse(algea.algeacmd(0));

    joystick.start().whileTrue(elevator1.runOnce(() -> elevator1.togglesetpoint()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
