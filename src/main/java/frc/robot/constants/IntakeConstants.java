package frc.robot.constants;

public final class IntakeConstants {
    public static final int intakeMotorCID = 20;

    public static final double intakeSpeedRPS = 55;

    public static final class PIDFeedforwardConstants {
        public static final double P = 0.0;
        public static final double I = 0;
        public static final double D = 0;
        public static final double S = .0175;
        public static final double V = 0.0105;
        public static final double A = 0;
        public static final double pidTolerance = 0.05;
    }

    // sim-only intake model
    public static final class SimConstants {
        public static final double gearing = 1.0;
        public static final double momentOfInertiaKgMetersSquared = 0.001;
    }
}
