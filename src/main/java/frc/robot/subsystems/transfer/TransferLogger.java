package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

final class TransferLogger {

    private TransferLogger() {}

    static void publish(Transfer transfer) {
        SmartDashboard.putData("Transfer", b ->
                b.addDoubleProperty("Transfer Speed", transfer::getTransferSpeed, null));
    }
}
