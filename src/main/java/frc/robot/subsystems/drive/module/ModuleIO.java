package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    class ModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMps = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d anglePosition = new Rotation2d();
        public Rotation2d angleAbsolutePosition = new Rotation2d();
        public double angleVelocityRps = 0.0;
        public double angleAppliedVolts = 0.0;
        public double angleCurrentAmps = 0.0;
    }

    default void updateInputs(ModuleIOInputs inputs) {}

    default void setDriveVelocity(double metersPerSecond) {}

    default void setDriveDutyCycle(double dutyCycle) {}

    default void setAnglePosition(Rotation2d angle) {}

    default void resetToAbsolute() {}

    default void stop() {}
}
