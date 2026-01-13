package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public interface HolonomicPathFollower {
    void driveRobotRelative(ChassisSpeeds robotOrientedSpeeds);

    Pose2d getPose();

    void setPose(Pose2d pose);

    ChassisSpeeds getRobotRelativeSpeeds();

    default void displayTrajectory(Trajectory trajectory) {
    }
}