package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Camera mount offsets from robot center, meters. TODO: measure camera_0 X/Y on the real robot before competition. */
public final class VisionConstants {
    public static double camera_0_OffsetX = 0;
    public static double camera_0_OffsetY = 0;
    public static double camera_0_OffsetZ = Units.inchesToMeters(18.5);

    public static double camera_1_OffsetX = Units.inchesToMeters(-2.5);
    public static double camera_1_OffsetY = Units.inchesToMeters(13.8);
    public static double camera_1_OffsetZ = Units.inchesToMeters(18.5);
}
