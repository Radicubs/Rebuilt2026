package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.opencv.photo.Photo;
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

    public SwerveDriveOdometry visionOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, Swerve.getInstance().getGyroYaw(), Swerve.getInstance().getModulePositions());;

    private static final Transform3d cameraOffset = new Transform3d(Constants.CameraConfig.cameraOffsetX, Constants.CameraConfig.cameraOffsetY, Constants.CameraConfig.cameraOffsetZ, new Rotation3d(0, 0, 0));

    public static PhotonVision getInstance() {
        if (instance == null) instance = new PhotonVision();
        return instance;
    }

    private final PhotonCamera camera;
    private PhotonPipelineResult result;

    private PhotonVision() {
        camera = new PhotonCamera("orange");
        Transform3d robotToCam = new Transform3d(Constants.CameraConfig.cameraOffsetX, Constants.CameraConfig.cameraOffsetY, Constants.CameraConfig.cameraOffsetZ, new Rotation3d());

        UsbCamera server = CameraServer.startAutomaticCapture(0);
        server.setResolution(640,480);

        SmartDashboard.putData("Photon Vision", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Has Tag", () -> getBestTag() != -1, null);
                builder.addIntegerProperty("Best Tag", () -> getBestTag(), null);
            }
        });
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
        return visionOdometry.getPoseMeters();
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
                    Optional<Pose3d> option = APRIL_TAG_LAYOUT.getTagPose(result.getBestTarget().getFiducialId());

                    if(option.isPresent()){
                        visionOdometry.resetPosition(
                                Swerve.getInstance().getGyroYaw(),
                                Swerve.getInstance().getModulePositions(),
                                option.get().plus(
                                        result.getBestTarget().getBestCameraToTarget().inverse()
                                                .plus(cameraOffset.inverse())).toPose2d());

                    }
                }
                else{
                    this.result = null;
                }
            }
        }

        visionOdometry.update(Swerve.getInstance().getGyroYaw(), Swerve.getInstance().getModulePositions());


    }
}
