package frc.robot.constants;

public final class TransferConstants {

    public static final int beltMotorCID = 18;

    public static final int beltMotorStallCurrentLimit = 40;
    public static final int beltMotorFreeCurrentLimit = 30;

    public static final double shootTransferSpeed = 60;
    public static final double intakeTransferSpeed = 20;

    public static final class TransferPIDFeedforwardConstants {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.03;
        public static final double kV = 0.0109;
        public static final double kA = 0.0;
    }

    // sim-only belt model
    public static final class SimConstants {
        public static final double gearing = 1.0;
        public static final double momentOfInertiaKgMetersSquared = 0.001;
    }
}
