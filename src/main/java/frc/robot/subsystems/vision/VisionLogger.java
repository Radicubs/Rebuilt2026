package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

final class VisionLogger {

    private VisionLogger() {}

    static void publish(PhotonVision vision) {
        SmartDashboard.putData("Photon Vision", b -> {
            b.addBooleanProperty("Has Tag", () -> vision.getBestTag() != -1, null);
            b.addIntegerProperty("Best Tag", vision::getBestTag, null);
        });
    }
}
