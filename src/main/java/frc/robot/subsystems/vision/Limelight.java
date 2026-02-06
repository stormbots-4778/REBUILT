package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;

public class Limelight {
    private final String name;

    public enum PositionTeam {
        BLUE, RED
    }

    private PositionTeam positionTeam;

    public Limelight(PositionTeam team) {
        name = "";
        positionTeam = team;
    }

    public Limelight(PositionTeam team, String llName) {
        name = llName;
        positionTeam = team;
    }

    public void setTeam(PositionTeam team) {
        positionTeam = team;
    }

    /**
     * Where I am, based on what I can see.
     */
    public Translation2d getBotPosition(double headingDegrees) {
        LimelightHelpers.SetRobotOrientation(name, headingDegrees, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate pose;
        if (positionTeam == PositionTeam.RED) {
            pose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name);
        } else {
            pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        }

        return pose.pose.getTranslation();
    }

    /**
     * The angular offset to whatever tag I can see.
     */
    public double tagAngle() {
        double tx = LimelightHelpers.getTX(name);
        return Math.toRadians(tx);
    }
}
