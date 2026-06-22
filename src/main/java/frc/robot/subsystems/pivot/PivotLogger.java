package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

final class PivotLogger {
    private PivotLogger() {}

    static void publish(Pivot pivot) {
        SmartDashboard.putData("Pivot", b -> {
            b.addDoubleProperty("Desired Pivot Angle", pivot::getDesiredAngle, null);
            b.addDoubleProperty("Current Pivot Angle", pivot::getPosition, null);
        });
    }
}
