package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class Limelight {
    private final String name;

    public Limelight(String llName) {
        name = llName;
    }

    /**
     * Where I am, based on what I can see.
     */
    public Pose2d getBotPose(double headingDegrees) {
        // LimelightHelpers.SetRobotOrientation(name, headingDegrees, 0, 0, 0, 0, 0);
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        return estimate.pose;
    }

    /**
     * The angular offset to whatever tag I can see.
     */
    public double tagAngle() {
        double tx = LimelightHelpers.getTX(name);
        return Math.toRadians(tx);
    }
}
