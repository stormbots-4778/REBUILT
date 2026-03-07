package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.FieldConfiguration;
import frc.robot.configuration.RobotConfiguration;
import frc.robot.configuration.RobotConfiguration.VisionConfig;
import frc.robot.subsystems.driving.Drivetrain;

public class Vision extends SubsystemBase {
    private static Pose3d applyOffset(Pose3d inputPose) {
        return inputPose.transformBy(new Transform3d(0, 0.5, 0, new Rotation3d()));
    }

    PhotonCamera cam1 = new PhotonCamera(RobotConfiguration.VisionConfig.cam1name);
    PhotonCamera cam2 = new PhotonCamera(RobotConfiguration.VisionConfig.cam1name);

    private void estimateInto(PhotonCamera cam, Transform3d offset, Drivetrain drive) {
        var result = cam.getLatestResult();
        if (!result.hasTargets())
            return;
        PhotonTrackedTarget target = result.getBestTarget();

        var pose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                FieldConfiguration.april.getTagPose(target.getFiducialId()).get(),
                offset);
        if (pose != null)
            drive.usePoseEstimate(applyOffset(pose).toPose2d());
    }

    public void passIntoDrivetrain(Drivetrain drive) {
        estimateInto(cam1, VisionConfig.cam1offset, drive);
        estimateInto(cam2, VisionConfig.cam2offset, drive);
    }
}
