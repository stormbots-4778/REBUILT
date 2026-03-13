package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.VisionConfig;
import frc.robot.subsystems.driving.Drivetrain;

public class Vision extends SubsystemBase {
    public Command zeroLimelightIMU = runOnce(() -> LimelightHelpers.SetIMUMode(VisionConfig.llname, 1))
            .andThen(Commands.waitSeconds(0.1))
            .andThen(() -> LimelightHelpers.SetRobotOrientation(VisionConfig.llname, 0, 0, 0, 0, 0, 0))
            .andThen(Commands.waitSeconds(0.1)).andThen(() -> LimelightHelpers.SetIMUMode(VisionConfig.llname, 2));

    public void passIntoDrivetrain(Drivetrain drive, Alliance alliance) {
        LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConfig.llname);
        if (llMeasurement.tagCount == 0)
            return;
        drive.usePoseEstimate(llMeasurement.pose, llMeasurement.timestampSeconds);
    }

    public Command setThrottling(BooleanSupplier throttle) {
        return run(() -> {
            if (throttle.getAsBoolean()) {
                LimelightHelpers.SetThrottle(VisionConfig.llname, 200);
            } else {
                LimelightHelpers.SetThrottle(VisionConfig.llname, 0);
            }
        });
    }
}
