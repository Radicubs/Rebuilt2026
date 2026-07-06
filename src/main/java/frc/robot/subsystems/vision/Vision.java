package frc.robot.subsystems.vision;

import frc.robot.constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

// AprilTag localization; fuses multi-tag estimates into the pose estimator. Real cameras only.
public class Vision extends SubsystemBase {

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) instance = new Vision();
        return instance;
    }

    private final PhotonCamera camera0 = new PhotonCamera("orange");
    private final PhotonPoseEstimator estimator0 = new PhotonPoseEstimator(APRIL_TAG_LAYOUT,
            new Transform3d(VisionConstants.camera_0_OffsetX, VisionConstants.camera_0_OffsetY,
                    VisionConstants.camera_0_OffsetZ, new Rotation3d(Rotation2d.k180deg)));
    private PhotonPipelineResult latestResult0;

    private final PhotonCamera camera1 = new PhotonCamera("juice");
    private final PhotonPoseEstimator estimator1 = new PhotonPoseEstimator(APRIL_TAG_LAYOUT,
            new Transform3d(VisionConstants.camera_1_OffsetX, VisionConstants.camera_1_OffsetY,
                    VisionConstants.camera_1_OffsetZ, new Rotation3d(Rotation2d.kCCW_90deg)));
    private PhotonPipelineResult latestResult1;

    private Vision() {
        VisionLogger.publish(this);
    }

    public int getBestTagId() {
        if (latestResult0 == null || !latestResult0.hasTargets()) return -1;
        return latestResult0.getBestTarget().getFiducialId();
    }

    boolean cam0Connected() { return camera0.isConnected(); }
    boolean cam1Connected() { return camera1.isConnected(); }

    @Override
    public void periodic() {
        latestResult0 = process(camera0, estimator0, latestResult0);
        latestResult1 = process(camera1, estimator1, latestResult1);
        VisionLogger.log(this);
    }

    private PhotonPipelineResult process(PhotonCamera camera, PhotonPoseEstimator estimator, PhotonPipelineResult last) {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) {
                last = null;
                continue;
            }
            last = result;
            Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
            estimate.ifPresent(est ->
                    Drive.getInstance().addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds));
        }
        return last;
    }
}
