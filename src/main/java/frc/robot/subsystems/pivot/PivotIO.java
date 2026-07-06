package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

interface PivotIO {

    @AutoLog
    class PivotIOInputs {
        public double positionRot = 0.0;
        public double appliedDuty = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(PivotIOInputs inputs) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setPosition(double positionRot) {}
}
