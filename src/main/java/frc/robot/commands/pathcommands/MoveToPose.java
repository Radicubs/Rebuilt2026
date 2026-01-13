package frc.robot.commands.pathcommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve;

public class MoveToPose extends PathFollowingCommand {
    public MoveToPose(Pose2d endPose, TrajectoryConstants constants) {
        super(Swerve.getInstance(), endPose, constants);
        System.out.println(endPose);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = controller.calculate(chassis.getPose(), endPose, 0, endPose.getRotation());
        double percent = constants.maxPathSpeed() / Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

        if(percent < 1){
            speeds.vxMetersPerSecond *= percent;
            speeds.vyMetersPerSecond *= percent;
        }

        speeds.omegaRadiansPerSecond = Math.min(constants.maxPathAngularSpeed(), speeds.omegaRadiansPerSecond);
        chassis.driveRobotRelative(speeds);
    }
}

