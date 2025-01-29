package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.utils.FieldConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for vision processing that provides different vision modes, measurement validation,
 * and data structures for vision measurements. Supports both MegaTag1 (MT1) and MegaTag2 (MT2)
 * vision systems.
 */
public class VisionUtil {
  // Configuration constants
  private static boolean BEFORE_MATCH = true; // Controls MT1-only usage before match
  public static final double FIELD_MARGIN =
      0.5; // Meters beyond field boundaries to accept measurements
  public static final double Z_MARGIN = 0.5; // Meters above/below field to accept measurements
  public static final double MT2_SPIN_MAX = 40.0; // Maximum rotation speed for MT2 measurements
  public static final double MIN_TAG_AREA = 0.05; // Minimum tag area to be accepted.

  // Vision measurement constants
  private static final double MA_VISION_STD_DEV_XY = 0.333;
  private static final double MA_VISION_STD_DEV_THETA = 5;
  public static final double MA_AMBIGUITY = 0.4;

  /**
   * Enum defining different vision processing modes with unique validation and measurement
   * calculation implementations.
   */
  public enum VisionMode {
    /**
     * Mode that rejects all vision measurements. Useful for testing or when vision should be
     * temporarily disabled.
     */
    NONE {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        return new VisionMeasurement(mt, VecBuilder.fill(0.0, 0.0, 0.0));
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        return false;
      }
    },

    /** Standard vision mode with distance-based standard deviations and basic validation checks. */
    MA {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        double xyStdDev = calculateXYStdDev(mt);
        // MT2 measurements don't provide reliable rotation data
        double thetaStdDev = mt.isMegaTag2() ? Double.MAX_VALUE : calculateThetaStdDev(mt);
        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        return !timeCheck(mt.poseEstimate())
            && !rotationSpeedCheck(mt.poseEstimate())
            && !ambiguityCheck(mt.poseEstimate())
            && !fieldBoundsCheck(mt.poseEstimate().pose());
      }

      private double calculateXYStdDev(PoseEstimate mt) {
        return MA_VISION_STD_DEV_XY * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }

      private double calculateThetaStdDev(PoseEstimate mt) {
        return MA_VISION_STD_DEV_THETA * Math.pow(mt.avgTagDist(), 2.0) / mt.tagCount();
      }
    },

    /** POOF vision mode with enhanced validation checks and measurement calculations. */
    POOF {
      @Override
      public VisionMeasurement getVisionMeasurement(PoseEstimate mt) {
        // Check if pose is within field bounds
        if (!fieldBoundsCheck(mt.pose())) {
          return new VisionMeasurement(
              mt, VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));
        }

        return calculatePoofMeasurement(mt);
      }

      @Override
      public boolean acceptVisionMeasurement(PoseObservation mt) {
        PoseEstimate poseEst = mt.poseEstimate();
        return !fieldBoundsCheck(poseEst.pose())
            && !rotationSpeedCheck(poseEst)
            && !tagAreaCheck(poseEst)
            && !ambiguityCheck(poseEst);
      }

      private VisionMeasurement calculatePoofMeasurement(PoseEstimate mt) {
        double xyStdDev;
        double thetaStdDev;

        // Calculate standard deviations based on number of tags and their properties
        boolean hasMultipleTags = mt.tagCount() >= 2;
        boolean hasLargeTagArea = mt.avgTagArea() > 0.1;

        if (hasMultipleTags) {
          // Multiple targets detected - highest confidence
          if (hasLargeTagArea) {
            xyStdDev = 0.2;
            thetaStdDev = Units.degreesToRadians(6.0);
          } else {
            xyStdDev = 1.2;
            thetaStdDev = Units.degreesToRadians(12.0);
          }
        } else if (mt.tagCount() == 1) {
          // Single target - evaluate based on area and position
          double poseDifference =
              mt.pose().getTranslation().getDistance(mt.pose().getTranslation());
          boolean isCloseToExpectedPose = poseDifference < 0.5;
          boolean isVeryCloseToExpectedPose = poseDifference < 0.3;

          if (mt.avgTagArea() > 0.8 && isCloseToExpectedPose) {
            // Large visible tag and close to expected position
            xyStdDev = 0.5;
            thetaStdDev = Units.degreesToRadians(12.0);
          } else if (hasLargeTagArea && isVeryCloseToExpectedPose) {
            // Smaller visible tag but very close to expected position
            xyStdDev = 1.0;
            thetaStdDev = Units.degreesToRadians(0.0);
          } else {
            // Lower confidence case
            xyStdDev = 2.0;
            thetaStdDev = Units.degreesToRadians(50.0);
          }
        } else {
          // No valid tags detected
          xyStdDev = Double.MAX_VALUE;
          thetaStdDev = Double.MAX_VALUE;
        }

        // MT2 measurements don't provide reliable rotation data
        if (mt.isMegaTag2()) {
          thetaStdDev = Double.MAX_VALUE;
        }

        return new VisionMeasurement(mt, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
      }
    };

    /**
     * Creates a vision measurement with calculated standard deviations.
     *
     * @param mt The pose estimate to process
     * @return A vision measurement with appropriate standard deviations
     */
    public abstract VisionMeasurement getVisionMeasurement(PoseEstimate mt);

    /**
     * Determines whether a vision measurement should be accepted.
     *
     * @param mt The pose observation to validate
     * @return True if the measurement should be accepted
     */
    public abstract boolean acceptVisionMeasurement(PoseObservation mt);
  }

  /** Record containing a pose estimate and its associated standard deviations. */
  public record VisionMeasurement(PoseEstimate poseEstimate, Vector<N3> visionMeasurementStdDevs) {}

  /**
   * Record containing collections of vision measurements and poses, categorized by acceptance
   * status.
   */
  public record VisionData(
      List<VisionMeasurement> measurements,
      List<Pose3d> tagPoses,
      List<Pose3d> acceptedTagPoses,
      List<Pose3d> rejectedTagPoses,
      List<Pose3d> robotPoses,
      List<Pose3d> acceptedPoses,
      List<Pose3d> rejectedPoses) {

    /** Creates an empty VisionData object. */
    public static VisionData empty() {
      return new VisionData(
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>(),
          new ArrayList<>());
    }

    /** Merges two lists of the same type. */
    private static <T> List<T> mergeLists(List<T> list1, List<T> list2) {
      ArrayList<T> merged = new ArrayList<>(list1);
      merged.addAll(list2);
      return merged;
    }

    /**
     * Combines this VisionData with another, merging all corresponding lists.
     *
     * @param other The VisionData to merge with
     * @return A new VisionData containing all elements from both objects
     */
    public VisionData merge(VisionData other) {
      return new VisionData(
          mergeLists(measurements, other.measurements),
          mergeLists(tagPoses, other.tagPoses),
          mergeLists(acceptedTagPoses, other.acceptedTagPoses),
          mergeLists(rejectedTagPoses, other.rejectedTagPoses),
          mergeLists(robotPoses, other.robotPoses),
          mergeLists(acceptedPoses, other.acceptedPoses),
          mergeLists(rejectedPoses, other.rejectedPoses));
    }
  }

  // Validation helper methods
  private static boolean timeCheck(PoseEstimate mt) {
    return mt.isMegaTag2() && beforeMatch();
  }

  private static boolean rotationSpeedCheck(PoseEstimate mt) {
    return Math.abs(mt.yawVelocityRadPerSec()) > MT2_SPIN_MAX;
  }

  private static boolean ambiguityCheck(PoseEstimate mt) {
    return mt.tagCount() == 1 && mt.ambiguity() > MA_AMBIGUITY;
  }

  private static boolean fieldBoundsCheck(Pose3d robotPose) {
    return robotPose.getX() < -FIELD_MARGIN
        || robotPose.getX() > FieldConstants.fieldLength + FIELD_MARGIN
        || robotPose.getY() < -FIELD_MARGIN
        || robotPose.getY() > FieldConstants.fieldWidth + FIELD_MARGIN
        || robotPose.getZ() < -Z_MARGIN
        || robotPose.getZ() > Z_MARGIN;
  }

  private static boolean tagAreaCheck(PoseEstimate mt) {
    return mt.avgTagArea() < MIN_TAG_AREA;
  }

  /** Checks if the robot is in the pre-match phase where only MT1 should be used. */
  public static boolean beforeMatch() {
    if (DriverStation.isEnabled()) {
      BEFORE_MATCH = false;
    }
    return BEFORE_MATCH;
  }
}
