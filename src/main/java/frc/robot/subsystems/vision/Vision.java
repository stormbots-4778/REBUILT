package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.VisionConfig;
import frc.robot.subsystems.driving.Drivetrain;

public class Vision extends SubsystemBase {
    public Command zeroLimelightIMU = runOnce(() -> LimelightHelpers.SetIMUMode(VisionConfig.ll4Name, 1))
            .andThen(Commands.waitSeconds(0.1))
            .andThen(() -> LimelightHelpers.SetRobotOrientation(VisionConfig.ll4Name, 0, 0, 0, 0, 0, 0))
            .andThen(Commands.waitSeconds(0.1)).andThen(() -> LimelightHelpers.SetIMUMode(VisionConfig.ll4Name, 2));

    private void useLL(String name, Drivetrain drive, boolean provideGyro) {
        LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (provideGyro)
            LimelightHelpers.SetRobotOrientation(name, drive.getHeadingDegrees(), 0, 0, 0, 0, 0);
        if (llMeasurement.tagCount == 0)
            return;
        drive.usePoseEstimate(llMeasurement.pose, llMeasurement.timestampSeconds);
    }

    public void passIntoDrivetrain(Drivetrain drive) {
        useLL(VisionConfig.ll4Name, drive, false);
        useLL(VisionConfig.ll2pName, drive, true);
    }
}
