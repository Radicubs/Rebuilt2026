package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

final class DriveLogger {

    private final Field2d field = new Field2d();

    DriveLogger(Drive drive) {
        SmartDashboard.putData(field);
        SmartDashboard.putData("Heading", b -> {
            b.setSmartDashboardType("Gyro");
            b.addDoubleProperty("Value", () -> (drive.getHeading() != null ? drive.getHeading().getDegrees() : 0), null);
        });
    }

    void log(Drive drive) {
        field.setRobotPose(drive.getPose());
        Logger.recordOutput("Drive/Pose", drive.getPose());
        Logger.recordOutput("Drive/Heading", drive.getHeading());
        Logger.recordOutput("Drive/GyroYaw", drive.getGyroYaw());
        Logger.recordOutput("Drive/ModuleStates", drive.getModuleStates());
        Logger.recordOutput("Drive/DesiredModuleStates", drive.getDesiredModuleStates());
        Logger.recordOutput("Drive/ChassisSpeeds", drive.getRobotRelativeSpeeds());
        Logger.recordOutput("Drive/DesiredChassisSpeeds", drive.getDesiredChassisSpeeds());
    }
}
