package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

interface TransferIO {

    @AutoLog
    class TransferIOInputs {
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(TransferIOInputs inputs) {}

    default void setDutyCycle(double dutyCycle) {}
}
