package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

final class PivotLogger {
    private PivotLogger() {}

    static void publish(Pivot pivot) {
        SmartDashboard.putData("Pivot", b -> {
            b.addDoubleProperty("Desired Pivot Angle", pivot::getDesiredAngle, null);
            b.addDoubleProperty("Current Pivot Angle", pivot::getPosition, null);
        });
    }

    static void log(Pivot pivot) {
        Logger.recordOutput("Pivot/Position", pivot.getPosition());
        Logger.recordOutput("Pivot/DesiredAngle", pivot.getDesiredAngle());
        Logger.recordOutput("Pivot/SetpointVelocity", pivot.getSetpointVelocity());
        Logger.recordOutput("Pivot/Error", pivot.getDesiredAngle() - pivot.getPosition());
        Logger.recordOutput("Pivot/AtGoal", pivot.atGoal());
        Logger.recordOutput("Pivot/MovingToTarget", pivot.isMovingToTarget());
        Logger.recordOutput("Pivot/AppliedDuty", pivot.getSpeed());
    }
}
