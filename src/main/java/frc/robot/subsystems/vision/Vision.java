package frc.robot.subsystems.vision;

import java.util.Optional;

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
        return inputPose.transformBy(new Transform3d(0, -0.4, 0, new Rotation3d()));
    }

    public record VisionResults(Optional<Pose3d> pose1, Optional<Pose3d> pose2) {
        public void useOn(Drivetrain dt) {
            pose1.ifPresent((p) -> dt.usePoseEstimate(applyOffset(p).toPose2d()));
            pose2.ifPresent((p) -> dt.usePoseEstimate(applyOffset(p).toPose2d()));
        }
    }

    PhotonCamera cam1 = new PhotonCamera(RobotConfiguration.VisionConfig.cam1name);
    PhotonCamera cam2 = new PhotonCamera(RobotConfiguration.VisionConfig.cam1name);

    private Optional<Pose3d> estimate(PhotonCamera cam, Transform3d offset) {
        var result = cam.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();
        PhotonTrackedTarget target = result.getBestTarget();

        return Optional.ofNullable(PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                FieldConfiguration.april.getTagPose(target.getFiducialId()).get(),
                offset));
    }

    public VisionResults getPoses() {
        return new VisionResults(estimate(cam1, VisionConfig.cam1offset), estimate(cam2, VisionConfig.cam2offset));
    }
}
