package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.PIDSwerve;
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

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final TunableController joystick =
      new TunableController(0).withControllerType(TunableControllerType.QUADRATIC);

  private final TunableController joystick2 =
      new TunableController(1).withControllerType(TunableControllerType.QUADRATIC);

  private final TunableController joystick3 =
      new TunableController(2).withControllerType(TunableControllerType.QUADRATIC);

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
  private shooter shoot = new shooter();
  private elevatorpid elevator1 = new elevatorpid();
  private algee algea = new algee();

  public RobotContainer() {
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);

         new Vision(
            drivetrain::addVisionData,
            new VisionIOPhotonVision("Cam1", 
                new Transform3d(new Translation3d(0.3,0.3,0),
                new Rotation3d(0, Math.toRadians(20), Math.toRadians(90))), drivetrain::getVisionParameters),
                new VisionIOPhotonVision("Cam2", 
                new Transform3d(new Translation3d(0.3,0.3,0),
                new Rotation3d(0, Math.toRadians(20), Math.toRadians(90))), drivetrain::getVisionParameters),
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

    NamedCommands.registerCommand("elevatoru",new Elevatorcmd(elevator1, 4));
    NamedCommands.registerCommand("elevatord",new Elevatorcmd(elevator1, 0));
    NamedCommands.registerCommand("shoot",shoot.AutonShoot(.07, 0));
    NamedCommands.registerCommand("intake",new SequentialCommandGroup(shoot.both(0.07, 0.25), shoot.wait(0.2, 0.1)));

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

    Command algeacmd = new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            algea.position(7.302734375), new Ballintake(algea, -0.8, -25)),
        algea.position(6.7));

    Command weirdshootingthing =(new ConditionalCommand(
            shoot.cmd(10),
            shoot.cmd3(5, 22),
            () -> Constants.getElevatorState() != Constants.Elevatorposition.Troph));

    Command highalgae  = new ParallelCommandGroup(new Elevatorcmd(elevator1,2),algea.algeacmd(12.09, 0.8));
    Command lowalgae  = new ParallelCommandGroup(new Elevatorcmd(elevator1,3),algea.algeacmd(12.09, 0.8)); //idk what to put for the setpoint as of now we havent tested it yet


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

    // joystick2
    //     .b()
    //     .whileTrue(new ParallelCommandGroup(
    // algea.cmd(12.095703125),0.8)).whileFalse(algea.position(0));

    // joystick2.a().whileTrue(algea.cmd(8.21875, -0.3)).whileFalse(algea.cmd(-0.3, -0.));
    joystick2
        .a()
        .whileTrue(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    algea.position(7.302734375), new Ballintake(algea, -0.8, -25)),
                algea.position(6.7)))
        .whileFalse(algea.algeacmd(0, 0));

    joystick2
        .a()
        .and(joystick2.y())
        .whileTrue(algea.algeacmd(7.302734375, 0.6))
        .whileFalse(algea.algeacmd(-0.3, 0));
    joystick2
        .x()
        .whileTrue(algea.algeacmd(12.095703125, 0.8))
        .whileFalse(algea.algeacmd(-0.3, -0.));
    joystick2.leftBumper().onTrue(algea.runOnce(() -> algea.resetalgea()));

    // joystick2
    //     .b()
    //     .whileTrue(elevator1.cmd2(27))
    //     .whileFalse(new ParallelCommandGroup(elevator1.cmd1(0)));
    joystick2.leftTrigger(0.2).whileTrue(algea.ion(-0.1)).whileFalse(algea.ion(0));

    // joystick2.rightBumper().whileTrue(elevator1.cmd2(2.3)).whileFalse(elevator1.cmd2(0));
    // joystick.a().onTrue(Commands.runOnce(() -> drivetrxain.resetPose(Pose2d.kZero)));
    // joystick
    //     .b()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED. Instructions
    // can be found here
    // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
    SwerveSetpointGen setpointGen =
        new SwerveSetpointGen(
                drivetrain.getChassisSpeeds(),
                drivetrain.getModuleStates(),
                drivetrain::getRotation)
            .withDeadband(MaxSpeed.times(0.1))
            .withRotationalDeadband(Constants.MaxAngularRate.times(0.1))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // joystick
    //     .x()
    //     .whileTrue(
    //         drivetrain.applyRequest(
    //             () ->
    //                 setpointGen
    //                     .withVelocityX(
    //                         MaxSpeed.times(
    //                             -joystick.getLeftY())) // Drive forward with negative Y (forward)
    //                     .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
    //
    // .withRotationalRate(Constants.MaxAngularRate.times(-joystick.getRightX()))
    //
    // .withOperatorForwardDirection(drivetrain.getOperatorForwardDirection())));

    // Custom Swerve Request that use ProfiledFieldCentricFacingAngle. Allows you to face specific
    // direction while driving
    ProfiledFieldCentricFacingAngle driveFacingAngle =
        new ProfiledFieldCentricFacingAngle(
                new TrapezoidProfile.Constraints(
                    Constants.MaxAngularRate.baseUnitMagnitude(),
                    Constants.MaxAngularRate.div(0.25).baseUnitMagnitude()))
            .withDeadband(MaxSpeed.times(0.1))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Set PID for ProfiledFieldCentricFacingAngle
    driveFacingAngle.HeadingController.setPID(7, 0, 0);
    // joystick
    //     .y()
    //     .whileTrue(
    //         drivetrain
    //             .runOnce(() -> driveFacingAngle.resetProfile(drivetrain.getRotation()))
    //             .andThen(
    //                 drivetrain.applyRequest(
    //                     () ->
    //                         driveFacingAngle
    //                             .withVelocityX(
    //                                 MaxSpeed.times(
    //                                     -joystick
    //                                         .getLeftY())) // Drive forward with negative Y
    // (forward)
    //                             .withVelocityY(MaxSpeed.times(-joystick.getLeftX()))
    //                             .withTargetDirection(new Rotation2d(1, 2)))));
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(elevator1.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(elevator1.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(elevator1.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(elevator1.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // joystick
    //     .a()
    //     .whileTrue(
    //         new SequentialCommandGroup(elevator1.Motionmagic(10.81214),
    // elevator1.pidup(10.81214)))
    //     .whileFalse(new ParallelCommandGroup(elevator1.piddown(0)));

    // joystick
    //     .leftTrigger(0.2)
    //     .whileTrue(drivetrain.driveToPose(new Pose2d(3, 2, new Rotation2d(12))));

    // joystick
    //     .b()
    //     .whileTrue(
    //         new SequentialCommandGroup(elevator1.Motionmagic(19.75566),
    // elevator1.pidup(19.75566)))
    //     .whileFalse(new ParallelCommandGroup(elevator1.piddown(0)));
    // joystick
    //     .y()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             elevator1.Motionmagic(30.091796875), elevator1.pidup(32.091796875)))
    //     .whileFalse(new ParallelCommandGroup(elevator1.piddown(0)));

    // joystick
    //     .back()
    //     .whileTrue(new SequentialCommandGroup(elevator1.Motionmagic(5), elevator1.pidup(5)))
    //     .whileFalse(new ParallelCommandGroup(elevator1.piddown(0)));

    // joystick.start().whileTrue(elevator1.resetenc());

    // joystick
    //     .y()
    //     .whileTrue(new Elevatorcmd(elevator1, 4)) // 32.0 or 30.0 depending on setpoints list
    //     .whileFalse(new Elevatorcmd(elevator1, 0)); // 0.0

    // joystick
    //     .back()
    //     .whileTrue(new Elevatorcmd(elevator1, 1)) // 5.0 or 17.0
    //     .whileFalse(new Elevatorcmd(elevator1, 0)); // 0.0

    // joystick
    //     .b()
    //     .whileTrue(new Elevatorcmd(elevator1, 3)) // 19.7 or 35.0
    //     .whileFalse(new Elevatorcmd(elevator1, 0)); // 0.0

    // joystick
    //     .a()
    //     .whileTrue(new Elevatorcmd(elevator1, 2)) // 10.8 or 30.0
    //     .whileFalse(new Elevatorcmd(elevator1, 0)); // 0.0

        

    // // joystick.rightTrigger(0.2).whileTrue(shoot.cmd3(5, 22));

    // joystick
    //     .rightBumper()
    //     .whileTrue(
    //         new ConditionalCommand(
    //             shoot.cmd(10),
    //             shoot.cmd3(5, 22),
    //             () -> Constants.getElevatorState() == Constants.Elevatorposition.Troph))
    //     .whileFalse(shoot.cmd(0));
    // // joystick.leftBumper().whileTrue(shoot.both(0.07, 0.25)).whileFalse(shoot.both(0, 0));
    // joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.resetgyro()));
    // // joystick.leftBumper().whileTrue(shoot.both(0.07, 0.25)).whileFalse(shoot.both(0, 0));
    // joystick
    //     .leftBumper()
    //     .whileTrue(new SequentialCommandGroup(shoot.both(0.07, 0.25), shoot.wait(0.2, 0.1)))
    //     .whileFalse(shoot.both(0, 0));
    // joystick.rightTrigger(0.2).whileTrue(elevator1.runOnce(() -> elevator1.togglesetpoint()));
    // joystick.leftBumper().whileTrue(shoot.both(0.07, 0.25)).whileFalse(shoot.both(0, 0));

    // // joystick.y().onTrue(elevator1.runOnce(() -> elevator1.resetenc()));
    // // joystick.start().onTrue(elevator1.runOnce(() -> elevator1.resetenc()));

    // // joystick.a().whileTrue(shoot.hopper(20)).whileFalse(shoot.hopper(0));













    joystick.leftTrigger(0.2).whileTrue(new ConditionalCommand(new HopperTriggeredShooterAndRumbleCommand(shoot, joystick),algeacmd, ()-> Constants.getElevatorState() != Constants.Elevatorposition.Troph)).whileFalse(new ParallelCommandGroup( shoot.both(0, 0)));

   

    joystick
        .rightTrigger(0.2)
        .whileTrue(
            new ConditionalCommand(
                weirdshootingthing,algea.algeacmd(7.302734375, 0.6), ()-> Constants.getRobotState() != Constants.RobotState.ALGEA)).whileFalse(new ParallelCommandGroup(algea.algeacmd(0, 0),shoot.cmd(0) ));

    joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.resetgyro()));

    joystick.a().whileTrue(new Elevatorcmd(elevator1, 1)).whileFalse(new Elevatorcmd(elevator1, 0));
    
    joystick.back().whileTrue(new Elevatorcmd(elevator1, 4)).whileFalse(new Elevatorcmd(elevator1, 0));    

    
    
    joystick.leftStick().whileTrue(new ConditionalCommand(new Elevatorcmd(elevator1, 3), highalgae, ()-> Constants.getRobotState() != Constants.RobotState.ALGEA)).whileFalse(new ParallelCommandGroup(new Elevatorcmd(elevator1, 0),algea.algeacmd(-0.3, 0)));

    joystick.rightStick().whileTrue(new ConditionalCommand(new Elevatorcmd(elevator1, 2), lowalgae, ()-> Constants.getRobotState() != Constants.RobotState.ALGEA)).whileFalse(new ParallelCommandGroup(new Elevatorcmd(elevator1, 0),algea.algeacmd(-0.3, 0)));

    joystick.start().whileTrue(elevator1.runOnce(() -> elevator1.togglesetpoint()));  

    Pose2d target = new Pose2d(new Translation2d(1,2),new Rotation2d(3));

    joystick.a().whileTrue(new PIDSwerve(drivetrain, target));
}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
