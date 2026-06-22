package frc.robot.subsystems.transfer;

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
}
