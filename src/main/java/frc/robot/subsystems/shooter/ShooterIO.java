package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public double leftVelocityRps = 0.0;
        public double rightVelocityRps = 0.0;
        public double topVelocityRps = 0.0;
        public double indexerVelocityRps = 0.0;

        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;
        public double topAppliedVolts = 0.0;
        public double indexerAppliedVolts = 0.0;

        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
        public double topCurrentAmps = 0.0;
        public double indexerCurrentAmps = 0.0;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    // mainRps drives both left and right flywheels
    default void setShooterSpeeds(double mainRps, double topRps, double indexerRps) {}

    default void setIndexerSpeed(double indexerRps) {}

    default void stop() {}
}
