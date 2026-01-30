package frc.robot.commands.pathcommands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

public class AlignToSpecific extends PathFollowingCommand {
    private int tagID;
    private HolonomicDriveController controller;
    private Pose2d tolerance;
    private TrajectoryConstants constants;

    private Transform2d transformFromTag;

    public AlignToSpecific(Transform2d transformFromTag, int tagID, TrajectoryConstants constants) {
        super(Swerve.getInstance(), Swerve.getInstance().getPose(), constants);
        this.constants = constants;
        this.tolerance = PathFollowingCommand.getTolerance(constants);
        this.controller = PathFollowingCommand.getController(constants);
        this.transformFromTag = transformFromTag;
        this.tagID = tagID;
        addRequirements(PhotonVision.getInstance(), Swerve.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        Transform3d toTarget = PhotonVision.getInstance().getTransformToTarget(tagID);

        if (toTarget == null || tagID == -1) {
            this.cancel();
            System.out.println("cancelled tag " + tagID);
            return;
        }

        Pose2d robotPose = new Pose2d(toTarget.getX(), toTarget.getY(), toTarget.getRotation().toRotation2d());
        chassis.setPose(robotPose);
        this.endPose = new Pose2d().plus(transformFromTag.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))));
        chassis.setPose(chassis.getPose());
        Swerve.getInstance().displayPose(endPose);

    }

    @Override
    public void execute() {
        Transform3d toTarget = PhotonVision.getInstance().getTransformToTarget(tagID);

        if (toTarget != null) {
            Pose2d robotPose = new Pose2d(toTarget.getX(), toTarget.getY(), toTarget.getRotation().toRotation2d());
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
        chassis.driveRobotRelative(speeds);
    }
}