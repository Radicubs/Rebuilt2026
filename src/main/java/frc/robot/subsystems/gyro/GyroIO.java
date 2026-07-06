package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yaw = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}

    default void reset() {}

    default void setSimYaw(Rotation2d yaw) {}
}
