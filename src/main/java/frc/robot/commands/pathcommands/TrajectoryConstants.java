package frc.robot.commands.pathcommands;

public record TrajectoryConstants(
        double maxPathSpeed,
        double maxPathAngularSpeed,
        double maxPathAcceleration,
        double maxPathAngularAcceleration,

        double xControllerkP,
        double xControllerkI,
        double xControllerkD,

        double yControllerkP,
        double yControllerkI,
        double yControllerkD,

        double thetaControllerkP,
        double thetaControllerkI,
        double thetaControllerkD,

        double xTolerance,
        double yTolerance,
        double rotTolerance,
        double velXTolerance,
        double velYTolerance,
        double rotVelTolerance
) {
    public static final TrajectoryConstants DEFAULTS = new TrajectoryConstants(4, 10,
            3.0, 14.0, 2.5, 0, 0,
            2.5, 0, 0, 1.5, 0, 0,
            0.05, 0.05, 0.05, 0.12, 0.12, 0.12);

    public static final TrajectoryConstants SLOW = new TrajectoryConstants(1, 10,
            3.0, 14.0, 1, 0, 0,
            1, 0, 0, 1.5, 0, 0,
            0.015, 0.015, 0.015, 0.1, 0.1, 0.1);

}
