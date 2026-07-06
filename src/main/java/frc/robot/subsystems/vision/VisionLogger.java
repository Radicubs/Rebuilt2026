package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

final class VisionLogger {

    private VisionLogger() {}

    static void publish(Vision vision) {
        SmartDashboard.putData("Photon Vision", b -> {
            b.addBooleanProperty("Has Tag", () -> vision.getBestTagId() != -1, null);
            b.addIntegerProperty("Best Tag", vision::getBestTagId, null);
        });
    }

    static void log(Vision vision) {
        Logger.recordOutput("Vision/HasTarget", vision.getBestTagId() != -1);
        Logger.recordOutput("Vision/BestTag", vision.getBestTagId());
        Logger.recordOutput("Vision/Cam0Connected", vision.cam0Connected());
        Logger.recordOutput("Vision/Cam1Connected", vision.cam1Connected());
    }
}
