package frc.robot.subsystems.pivot;

public final class PivotConstants {
    public static final int pivotMotorCID = 19;
    public static final int pivotMotorStallCurrentLimit = 20;
    public static final int pivotMotorFreeCurrentLimit = 10;

    public static final double downPos = 0.080555;
    public static final double upPos = -0.3662683069705963;
    public static final double middlePos = -.1;
    public static final double pivotFinalVelocity = 0.0;

    public static final class PIDFeedforwardConstants {
        public static final double P = 4;
        public static final double I = 0;
        public static final double D = 0;
        public static final double S = 0;
        public static final double V = 0.1;
        public static final double G = -0.05;
        public static final double pidTolerance = 0.01;
    }
}
