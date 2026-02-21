package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Navx extends SubsystemBase {
    // Subsystem for the Navx gyro. Has methods to get roll, pitch, and yaw of the gyro, along with resetting
    // some axes to specific values

    private final AHRS gyro;
    private static Navx instance;

    public static Navx getInstance() {
        if (instance == null) instance = new Navx();
        return instance;
    }

    private Navx() {
        gyro = new AHRS(AHRS.NavXComType.kUSB1);
        gyro.reset();
    }

    public double getYaw() {
        return -gyro.getYaw();
    } // negated because gyro is upside down

    public double getRoll() {
        return gyro.getRoll();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void reset() {
        gyro.reset();
    }
}