package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

class GyroIOSim implements GyroIO {

    private Rotation2d yaw = new Rotation2d();

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yaw = yaw;
        inputs.yawVelocityRadPerSec = 0.0;
    }

    @Override
    public void reset() {
        yaw = new Rotation2d();
    }

    @Override
    public void setSimYaw(Rotation2d yaw) {
        this.yaw = yaw;
    }
}
