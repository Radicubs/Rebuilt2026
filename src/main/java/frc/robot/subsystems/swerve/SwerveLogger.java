package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

final class SwerveLogger {

    private final Field2d field = new Field2d();

    SwerveLogger(Swerve swerve) {
        SmartDashboard.putData(field);
        SmartDashboard.putData("Heading", b -> {
            b.setSmartDashboardType("Gyro");
            b.addDoubleProperty("Value", () -> (swerve.getHeading() != null ? swerve.getHeading().getDegrees() : 0), null);
        });
    }

    /** Called every loop from {@code Swerve.periodic()} to track the robot pose. */
    void update(Pose2d pose) {
        field.setRobotPose(pose);
    }

    void setTrajectory(Trajectory trajectory) {
        field.getObject("trajectory").setTrajectory(trajectory);
    }

    void setPose(Pose2d pose) {
        field.getObject("poses").setPose(pose);
    }
}
