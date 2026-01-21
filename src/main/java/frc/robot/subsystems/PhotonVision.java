package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class PhotonVision extends SubsystemBase {

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static PhotonVision instance;

    private static final Transform3d cameraOffset = new Transform3d(Constants.CameraConfig.cameraOffsetX, Constants.CameraConfig.cameraOffsetY, Constants.CameraConfig.cameraOffsetZ, new Rotation3d(0, 0, 0));

    //private PhotonPoseEstimator poseEstimator;

    public static PhotonVision getInstance() {
        if (instance == null) instance = new PhotonVision();
        return instance;
    }

    private final PhotonCamera camera;
    private final Timer timer;

    private PhotonPipelineResult result;

    private PhotonVision() {
        timer = new Timer();
        camera = new PhotonCamera("raspberry");
        Transform3d robotToCam = new Transform3d(Constants.CameraConfig.cameraOffsetX, Constants.CameraConfig.cameraOffsetY, Constants.CameraConfig.cameraOffsetZ, new Rotation3d());
        //poseEstimator = new PhotonPoseEstimator(APRIL_TAG_LAYOUT, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
    }

    public Transform3d getTransformToTarget(int targetID) {
        if (result == null || !result.hasTargets())
            return null;

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == targetID) {
                return target.getBestCameraToTarget();
            }
        }
        return null;
    }

    public Pose2d getRobotFieldPose() {
        if(result == null) return null;
        Optional<Pose3d> option = APRIL_TAG_LAYOUT.getTagPose(result.getBestTarget().getFiducialId());

        if(option.isPresent()){
            return option.get().plus(result.getBestTarget()
                            .getBestCameraToTarget().inverse()
                            .plus(cameraOffset.inverse())).toPose2d();
        }

        return null;
    }

    public int getBestTag() {
        if(result == null) return -1;
        if (result.hasTargets())
            return result.getBestTarget().getFiducialId();
        return -1;
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if(!results.isEmpty()){
            for(PhotonPipelineResult res : results){
                if(res.hasTargets()){
                    this.result = res;
                    timer.reset();
                    timer.start();
                }
            }
        }

        if(timer.get() > 2 && result != null){
            result = null;
            System.out.println("Tag expired");
        }
    }
}
