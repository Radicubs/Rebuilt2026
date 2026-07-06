package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldConstants {
    /* Starting Positions */
    public static final Pose2d rightStart = new Pose2d(3.651, 0.613, new Rotation2d());
    public static final Pose2d middleStart =  new Pose2d(3.651, 3.964, new Rotation2d());
    public static final Pose2d leftStart = new Pose2d(3.651, 7.444, new Rotation2d());

    /* Positions */
    public static final Pose2d hubCenterBlue = new Pose2d(4.615, 4.049, new Rotation2d());
    public static final Pose2d hubCenterRed = new Pose2d(11.945, 4.041, new Rotation2d());
    public static final Pose2d hub = new Pose2d(2.318, 3.964, new Rotation2d());

    public static final Pose2d depotLeft = new Pose2d(0.429, 7.1408, Rotation2d.fromDegrees(90));
    public static final Pose2d depotRight = new Pose2d(0.429, 4.866, Rotation2d.fromDegrees(90));
    public static final Pose2d outpost = new Pose2d(0.429, 0.713, Rotation2d.fromDegrees(-90));

    public static final Pose2d rightLadder = new Pose2d(1.348, 2.864, Rotation2d.fromDegrees(180));
    public static final Pose2d middleLadder = new Pose2d(0.766, 3.731, new Rotation2d());
    public static final Pose2d leftLadder = new Pose2d(0.766, 4.546, new Rotation2d());
}
