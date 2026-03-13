package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.VisionConfig;
import frc.robot.subsystems.driving.Drivetrain;

public class Vision extends SubsystemBase {
    public Command zeroLimelightIMU = runOnce(() -> {
        LimelightHelpers.SetIMUMode(VisionConfig.llname, 1);
        LimelightHelpers.SetRobotOrientation(VisionConfig.llname, 0, 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode(VisionConfig.llname, 4);
    });

    public void passIntoDrivetrain(Drivetrain drive, Alliance alliance) {
        double yaw = drive.getHeadingDegrees();
        LimelightHelpers.SetRobotOrientation(VisionConfig.llname, yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConfig.llname);
        if (llMeasurement.tagCount == 0)
            return;
        drive.usePoseEstimate(llMeasurement.pose, llMeasurement.timestampSeconds);
    }
}
