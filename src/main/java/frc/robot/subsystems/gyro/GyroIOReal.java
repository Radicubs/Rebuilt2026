package frc.robot.subsystems.gyro;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

class GyroIOReal implements GyroIO {

    private final AHRS navx = new AHRS(AHRS.NavXComType.kUSB1);

    GyroIOReal() {
        navx.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.yaw = Rotation2d.fromDegrees(-navx.getYaw()); // navx yaw is cw-positive
        inputs.yawVelocityRadPerSec = -Units.degreesToRadians(navx.getRate());
    }

    @Override
    public void reset() {
        navx.reset();
    }
}
