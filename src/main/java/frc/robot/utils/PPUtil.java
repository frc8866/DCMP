package frc.robot.utils;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class PPUtil {

  private static final Alert moiAlert = new Alert("MOI Config Mismatch", AlertType.kError);
  private static final Alert massAlert = new Alert("Mass Config Mismatch", AlertType.kError);
  private static final Alert torqueAlert = new Alert("Torque Friction Mismatch", AlertType.kError);
  private static final Alert currentAlert =
      new Alert("Drive Current Limit Mismatch", AlertType.kError);
  private static final Alert motorAlert =
      new Alert("Drive Motor Config Mismatch", AlertType.kError);
  private static final Alert velocityAlert =
      new Alert("Max Drive Velocity Mismatch", AlertType.kError);
  private static final Alert cofAlert = new Alert("Wheel COF Mismatch", AlertType.kError);
  private static final Alert radiusAlert = new Alert("Wheel Radius Mismatch", AlertType.kError);
  private static final Alert locationAlert =
      new Alert("Module Location Mismatch", AlertType.kError);

  public static void compareConfigs(RobotConfig config1, RobotConfig config2) {
    if (config1 == null
        || config2 == null
        || config1.moduleLocations.length != config2.moduleLocations.length) {
      new Alert("Config Error", AlertType.kError)
          .setText("One of the configs is null or array lengths differ");
      return;
    }

    if (config1.MOI != config2.MOI) {
      moiAlert.setText(String.format("MOI: %.2f vs %.2f", config1.MOI, config2.MOI));
      moiAlert.set(true);
    }

    if (config1.massKG != config2.massKG) {
      massAlert.setText(String.format("Mass: %.2f vs %.2f kg", config1.massKG, config2.massKG));
      massAlert.set(true);
    }

    if (config1.maxTorqueFriction != config2.maxTorqueFriction) {
      torqueAlert.setText(
          String.format(
              "Torque Friction: %.2f vs %.2f",
              config1.maxTorqueFriction, config2.maxTorqueFriction));
      torqueAlert.set(true);
    }

    if (config1.moduleConfig.driveCurrentLimit != config2.moduleConfig.driveCurrentLimit) {
      currentAlert.setText(
          String.format(
              "Drive Current Limit: %.2f vs %.2f",
              config1.moduleConfig.driveCurrentLimit, config2.moduleConfig.driveCurrentLimit));
      currentAlert.set(true);
    }

    if (!config1.moduleConfig.driveMotor.equals(config2.moduleConfig.driveMotor)) {
      motorAlert.setText("Drive Motor configurations differ");
      motorAlert.set(true);
    }

    if (config1.moduleConfig.maxDriveVelocityMPS != config2.moduleConfig.maxDriveVelocityMPS) {
      velocityAlert.setText(
          String.format(
              "Max Drive Velocity: %.2f vs %.2f m/s",
              config1.moduleConfig.maxDriveVelocityMPS, config2.moduleConfig.maxDriveVelocityMPS));
      velocityAlert.set(true);
    }

    if (config1.moduleConfig.wheelCOF != config2.moduleConfig.wheelCOF) {
      cofAlert.setText(
          String.format(
              "Wheel COF: %.2f vs %.2f",
              config1.moduleConfig.wheelCOF, config2.moduleConfig.wheelCOF));
      cofAlert.set(true);
    }

    if (config1.moduleConfig.wheelRadiusMeters != config2.moduleConfig.wheelRadiusMeters) {
      radiusAlert.setText(
          String.format(
              "Wheel Radius: %.3f vs %.3f m",
              config1.moduleConfig.wheelRadiusMeters, config2.moduleConfig.wheelRadiusMeters));
      radiusAlert.set(true);
    }
    StringBuilder locationDifferences = new StringBuilder();
    boolean hasLocationDifferences = false;

    for (int i = 0; i < config1.moduleLocations.length; i++) {
      if (!config1.moduleLocations[i].equals(config2.moduleLocations[i])) {
        if (hasLocationDifferences) {
          locationDifferences.append(" | ");
        }
        locationDifferences.append(
            String.format(
                "Module %d: %s vs %s",
                i, config1.moduleLocations[i].toString(), config2.moduleLocations[i].toString()));
        hasLocationDifferences = true;
      }
    }

    if (hasLocationDifferences) {
      locationAlert.setText(locationDifferences.toString());
      locationAlert.set(true);
    }
  }
}
