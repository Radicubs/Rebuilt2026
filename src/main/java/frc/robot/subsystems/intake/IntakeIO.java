package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setDutyCycle(double dutyCycle) {}
}
