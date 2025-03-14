// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.drive.ChassisSpeeds;
// import frc.robot.subsystems.SwerveSubsystem;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// public class DriveToAprilTag2DCommand extends CommandBase {
//     private final SwerveSubsystem swerve;
//     private final PhotonCamera camera;

//     // Robot/camera geometry
//     private final double cameraHeight;      // meters
//     private final double targetHeight;      // meters
//     private final double cameraPitchAngle;  // degrees

//     // Desired distance to maintain from the target
//     private final double targetDistance = 1.0; // meters

//     // How many degrees we want to offset from center
//     private final double offsetDegrees;

//     // PID controllers for distance and rotation
//     private final PIDController distancePID = new PIDController(0.1, 0.0, 0.01);
//     private final PIDController rotationPID = new PIDController(0.02, 0.0, 0.001);

//     public DriveToAprilTag2DCommand(
//         SwerveSubsystem swerve,
//         PhotonCamera camera,
//         double cameraHeight,
//         double targetHeight,
//         double cameraPitchAngle,
//         double offsetDegrees
//     ) {
//         this.swerve = swerve;
//         this.camera = camera;
//         this.cameraHeight = cameraHeight;
//         this.targetHeight = targetHeight;
//         this.cameraPitchAngle = cameraPitchAngle;
//         this.offsetDegrees = offsetDegrees;

//         addRequirements(swerve);
//     }

//     @Override
//     public void initialize() {
//         distancePID.reset();
//         rotationPID.reset();
//     }

//     @Override
//     public void execute() {
//         PhotonPipelineResult result = camera.getLatestResult();
//         if (result.hasTargets()) {
//             PhotonTrackedTarget bestTarget = result.getBestTarget();

//             // 1. Horizontal offset (yaw) from crosshair to target (degrees)
//             double yaw = bestTarget.getYaw();

//             // Subtract offset so we don't aim exactly at center
//             double adjustedYaw = yaw - offsetDegrees;

//             // 2. Vertical offset (pitch) from crosshair to target (degrees)
//             double pitch = bestTarget.getPitch();

//             // 3. Approximate distance (2D geometry):
//             //    distance = (targetHeight - cameraHeight)
//             //               / tan( cameraPitchAngle + pitch )
//             double pitchTotalDeg = cameraPitchAngle + pitch;
//             double pitchTotalRad = Math.toRadians(pitchTotalDeg);
//             double distanceMeters = (targetHeight - cameraHeight) / Math.tan(pitchTotalRad);

//             // 4. Calculate how far off we are from our desired distance
//             double distanceError = distanceMeters - targetDistance;

//             // 5. Run each PID
//             double forwardPower  = distancePID.calculate(distanceError, 0.0);
//             double rotationPower = rotationPID.calculate(adjustedYaw, 0.0);

//             // 6. Clamp speeds (tune for your robot)
//             double maxLinearSpeed = 0.5;  // m/s
//             double maxRotSpeed    = 1.0;  // rad/s (or however you interpret rotation)
//             forwardPower  = MathUtil.clamp(forwardPower, -maxLinearSpeed, maxLinearSpeed);
//             rotationPower = MathUtil.clamp(rotationPower, -maxRotSpeed,   maxRotSpeed);

//             // 7. Drive with swerve (xSpeed = forward/back, ySpeed=0, rot=rotation)
//             swerve.drive(new ChassisSpeeds(forwardPower, 0.0, rotationPower), false);

//         } else {
//             // No targets detected: stop or search
//             swerve.stop();
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         // Optionally, stop if close enough to yaw=offset & targetDistance
//         PhotonPipelineResult result = camera.getLatestResult();
//         if (result.hasTargets()) {
//             PhotonTrackedTarget bestTarget = result.getBestTarget();
//             double yawError = bestTarget.getYaw() - offsetDegrees;

//             double pitchTotalDeg = cameraPitchAngle + bestTarget.getPitch();
//             double pitchTotalRad = Math.toRadians(pitchTotalDeg);
//             double distanceMeters = (targetHeight - cameraHeight) / Math.tan(pitchTotalRad);
//             double distanceError = distanceMeters - targetDistance;

//             boolean closeYaw = Math.abs(yawError) < 2.0;        // within 2 degrees
//             boolean closeDist = Math.abs(distanceError) < 0.1;  // within 0.1 m
//             return (closeYaw && closeDist);
//         }
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerve.stop();
//     }
// }
