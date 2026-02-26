package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.configuration.FieldConfiguration;
import frc.robot.configuration.RobotConfiguration;
import frc.robot.configuration.RobotConfiguration.VisionConfig;

public class Photon {
    PhotonCamera cam1 = new PhotonCamera(RobotConfiguration.VisionConfig.cam1name);

    public Pose3d getBotPose() {
        var result = cam1.getLatestResult();
        if (!result.hasTargets())
            return null;
        PhotonTrackedTarget target = result.getBestTarget();
        System.out.println(target);

        return PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                FieldConfiguration.april.getTagPose(target.getFiducialId()).get(),
                VisionConfig.cam1offset);
    }
}
