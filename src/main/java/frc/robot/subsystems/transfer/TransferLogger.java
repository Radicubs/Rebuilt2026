package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

final class TransferLogger {

    private TransferLogger() {}

    static void publish(Transfer transfer) {
        SmartDashboard.putData("Transfer", b ->
                b.addDoubleProperty("Transfer Speed", transfer::getTransferSpeed, null));
    }

    static void log(Transfer transfer) {
        Logger.recordOutput("Transfer/Setpoint", transfer.getSetpoint());
        Logger.recordOutput("Transfer/Error", transfer.getSetpoint() - transfer.getTransferSpeed());
        Logger.recordOutput("Transfer/Active", transfer.isActive());
    }
}
