package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.field.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class PhotonVision extends SubsystemBase {

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private static PhotonVision instance;


    public static PhotonVision getInstance() {
        if (instance == null) instance = new PhotonVision();
        return instance;
    }

    private final PhotonCamera camera_0;
    private PhotonPipelineResult cam_0_result;
    private PhotonPoseEstimator cam_0_photonEstimator;
    private static final Transform3d camera_0_Offset = new Transform3d(VisionConstants.camera_0_OffsetX, VisionConstants.camera_0_OffsetY, VisionConstants.camera_0_OffsetZ, new Rotation3d(Rotation2d.k180deg));


    private final PhotonCamera camera_1;
    private  PhotonPipelineResult cam_1_result;
    private PhotonPoseEstimator cam_1_photonEstimator;
    private static final Transform3d camera_1_Offset = new Transform3d(VisionConstants.camera_1_OffsetX, VisionConstants.camera_1_OffsetY, VisionConstants.camera_1_OffsetZ, new Rotation3d(Rotation2d.kCCW_90deg));


    private PhotonVision() {
        camera_0 = new PhotonCamera("orange");
        cam_0_photonEstimator = new PhotonPoseEstimator(APRIL_TAG_LAYOUT, camera_0_Offset);

        camera_1 = new PhotonCamera("juice");
        cam_1_photonEstimator = new PhotonPoseEstimator(APRIL_TAG_LAYOUT, camera_1_Offset);


        VisionLogger.publish(this);
    }

    public Transform3d getTransformToTarget(int targetID) {
        if (cam_0_result == null || !cam_0_result.hasTargets())
            return null;

        for (PhotonTrackedTarget target : cam_0_result.getTargets()) {
            if (target.getFiducialId() == targetID) {
                return target.getBestCameraToTarget();
            }
        }
        return null;
    }

    public int getBestTag() {
        if(cam_0_result == null) return -1;
        if (cam_0_result.hasTargets())
            return cam_0_result.getBestTarget().getFiducialId();
        return -1;
    }

    public double getDistanceFromHub(){
        Pose2d curPose = Swerve.getInstance().getPose();
        double dist = curPose.getTranslation().getDistance(FieldConstants.hubCenterBlue.getTranslation());;
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)){
                dist = curPose.getTranslation().getDistance(FieldConstants.hubCenterRed.getTranslation());
            }
        }

        return dist;
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> cam_0_results = camera_0.getAllUnreadResults();
        Optional<EstimatedRobotPose> cam_0_visionEst;

        if(!cam_0_results.isEmpty()){
            for(PhotonPipelineResult res : cam_0_results){
                if(res.hasTargets()){
                    this.cam_0_result = res;

                    cam_0_visionEst = cam_0_photonEstimator.estimateCoprocMultiTagPose(cam_0_result);
                    if(cam_0_visionEst.isPresent()) {
                        Swerve.getInstance().addVisionMeasurement(cam_0_visionEst.get().estimatedPose.toPose2d(), cam_0_visionEst.get().timestampSeconds);
                    }
                }
                else{
                    this.cam_0_result = null;
                }
            }
        }

        List<PhotonPipelineResult> cam_1_results = camera_1.getAllUnreadResults();
        Optional<EstimatedRobotPose> cam_1_visionEst;

        if(!cam_1_results.isEmpty()){
            for(PhotonPipelineResult res: cam_1_results){
                if(res.hasTargets()){
                    this.cam_1_result = res;

                    cam_1_visionEst = cam_1_photonEstimator.estimateCoprocMultiTagPose(cam_1_result);
                    if(cam_1_visionEst.isPresent()) {
                        Swerve.getInstance().addVisionMeasurement(cam_1_visionEst.get().estimatedPose.toPose2d(), cam_1_visionEst.get().timestampSeconds);
                    }
                }
                else{
                    this.cam_1_result = null;
                }
            }
        }
    }
}
