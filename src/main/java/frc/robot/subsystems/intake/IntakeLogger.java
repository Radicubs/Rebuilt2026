package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

final class IntakeLogger {

    private IntakeLogger() {}

    static void publish(Intake intake) {
        SmartDashboard.putData("Intake", b ->
                b.addDoubleProperty("Intake Speed", intake::getVelocity, null));
    }

    static void log(Intake intake) {
        Logger.recordOutput("Intake/Setpoint", intake.getSetpoint());
        Logger.recordOutput("Intake/Error", intake.getSetpoint() - intake.getVelocity());
        Logger.recordOutput("Intake/AtSetpoint", intake.atSetpoint());
        Logger.recordOutput("Intake/PidEnabled", intake.isPidEnabled());
    }
}
