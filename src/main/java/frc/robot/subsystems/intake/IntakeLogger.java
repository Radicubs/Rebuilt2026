package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

final class IntakeLogger {

    private IntakeLogger() {}

    static void publish(Intake intake) {
        SmartDashboard.putData("Intake", b ->
                b.addDoubleProperty("Intake Speed", intake::getVelocity, null));
    }
}
