package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Gyro extends SubsystemBase {

    private static Gyro instance;

    private final GyroIO io;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public static Gyro getInstance() {
        if (instance == null) {instance = new Gyro();}
        return instance;
    }

    private Gyro() {
        io = RobotBase.isSimulation() ? new GyroIOSim() : new GyroIOReal();
        io.reset();
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Gyro", inputs);
    }

    public Rotation2d getYaw() {
        return inputs.yaw;
    }

    public void reset() {
        io.reset();
    }

    public void setSimYaw(Rotation2d yaw) {
        io.setSimYaw(yaw);
    }
}
