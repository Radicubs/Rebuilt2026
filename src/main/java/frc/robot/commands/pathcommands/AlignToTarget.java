package frc.robot.commands.pathcommands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.TrajectoryConstants;

import java.util.Optional;

public class AlignToTarget extends PathFollowingCommand {
    private int tagID;
    private HolonomicDriveController controller;
    private TrajectoryConstants constants;

    private Transform2d transformFromTag;

    public AlignToTarget(Transform2d transformFromTag, TrajectoryConstants constants) {
        super(Swerve.getInstance(), Swerve.getInstance().getPose().plus(new Transform2d(1, 0, new Rotation2d())), constants);
        this.constants = constants;
        this.tolerance = PathFollowingCommand.getTolerance(constants);
        this.controller = PathFollowingCommand.getController(constants);
        this.transformFromTag = transformFromTag;
        addRequirements(PhotonVision.getInstance(), Swerve.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        tagID = PhotonVision.getInstance().getBestTag();
        Pose2d robotPose = PhotonVision.getInstance().getRobotFieldPose();
        Optional<Pose3d> tagPose = PhotonVision.APRIL_TAG_LAYOUT.getTagPose(tagID);
        if (tagID == -1 || robotPose == null || tagPose.isEmpty()) {
            this.cancel();
            System.out.println("cancelled tag " + tagID);
            return;
        }

        Swerve.getInstance().setPose(robotPose);
        this.endPose = tagPose.get().toPose2d().plus(transformFromTag.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))));
        System.out.println(tagPose);
        System.out.println(robotPose);
        Swerve.getInstance().setPose(robotPose);
        Swerve.getInstance().displayPose(endPose);

    }

    @Override
    public void execute() {
        Pose2d robotPose = PhotonVision.getInstance().getRobotFieldPose();

        if (robotPose != null) {
            Swerve.getInstance().setPose(robotPose);
        } else {
            System.out.println("Robot Pose is null");
        }

        ChassisSpeeds speeds = controller.calculate(Swerve.getInstance().getPose(), endPose, 0, endPose.getRotation());

        double percent = constants.maxPathSpeed() / Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

        if(percent < 1){
            speeds.vxMetersPerSecond *= percent;
            speeds.vyMetersPerSecond *= percent;
        }

        speeds.omegaRadiansPerSecond = Math.min(constants.maxPathAngularSpeed(), speeds.omegaRadiansPerSecond);
        //chassis.driveRobotRelative(speeds);
        SmartDashboard.putNumber("X m/s", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Y m/s", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Theta m/s", speeds.omegaRadiansPerSecond);


    }
}