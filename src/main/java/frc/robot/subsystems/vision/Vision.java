package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.VisionConfig;
import frc.robot.subsystems.driving.Drivetrain;

public class Vision extends SubsystemBase {
    private Alliance m_alliance = Alliance.Red;

    private double getAllianceAngleOffset() {
        System.out.println("GETTING THE ALLIANCE ANGLE OFFSET ITS " + m_alliance + "!!!!");
        return m_alliance == Alliance.Blue ? 180 : 0;
    }

    public Command zeroLimelightIMU() {
        return runOnce(() -> LimelightHelpers.SetIMUMode(VisionConfig.ll4Name, 1))
                .andThen(Commands.waitSeconds(0.1))
                .andThen(() -> LimelightHelpers.SetRobotOrientation(VisionConfig.ll4Name,
                        getAllianceAngleOffset(), 0, 0, 0, 0, 0))
                .andThen(Commands.waitSeconds(0.1)).andThen(() -> LimelightHelpers.SetIMUMode(VisionConfig.ll4Name, 2));
    }

    private void useLL(String name, Drivetrain drive, boolean provideGyro) {
        LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (provideGyro)
            LimelightHelpers.SetRobotOrientation(name, drive.getHeadingDegrees() + getAllianceAngleOffset(), 0, 0, 0, 0,
                    0);
        if (llMeasurement.tagCount == 0)
            return;
        drive.usePoseEstimate(llMeasurement.pose, llMeasurement.timestampSeconds);
    }

    public void setAlliance(Alliance alliance) {
        m_alliance = alliance;
    }

    public void passIntoDrivetrain(Drivetrain drive) {
        useLL(VisionConfig.ll4Name, drive, false);
        useLL(VisionConfig.ll2pName, drive, true);
    }
}
