package frc.robot.configuration;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConfiguration {
    // page 5 of
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // things converted to meters
    public static final Translation2d RED_GOAL_CENTER = new Translation2d(11.9, 4.02);
    public static final Translation2d BLUE_GOAL_CENTER = new Translation2d(4.6, 4.02);

    public static Translation2d getGoalPosition(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return BLUE_GOAL_CENTER;
        } else {
            return RED_GOAL_CENTER;
        }
    }

    public static final AprilTagFieldLayout april = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
}
