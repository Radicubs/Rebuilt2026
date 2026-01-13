package frc.robot.commands.pathcommands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HolonomicPathFollower;
import frc.robot.subsystems.Swerve;

public abstract class PathFollowingCommand extends Command {
    protected final HolonomicPathFollower chassis;
    protected HolonomicDriveController controller;
    protected final Timer timer;
    protected Pose2d endPose, tolerance;
    protected final TrajectoryConstants constants;

    public static HolonomicDriveController getController(TrajectoryConstants constants) {
        return new HolonomicDriveController(
                new PIDController(constants.xControllerkP(), constants.xControllerkI(), constants.xControllerkD()),
                new PIDController(constants.yControllerkP(), constants.yControllerkI(), constants.yControllerkD()),
                new ProfiledPIDController(constants.thetaControllerkP(), constants.thetaControllerkI(),
                        constants.thetaControllerkD(), new TrapezoidProfile.Constraints(
                        constants.maxPathAngularSpeed(), constants.maxPathAngularAcceleration())));
    }

    public static Pose2d getTolerance(TrajectoryConstants constants) {
        return new Pose2d(constants.xTolerance(), constants.yTolerance(),
                Rotation2d.fromRadians(constants.rotTolerance()));
    }

    public <T extends SubsystemBase & HolonomicPathFollower>
    PathFollowingCommand(T chassis, Pose2d endPose, final TrajectoryConstants constants) {
        this.constants = constants;
        this.chassis = chassis;
        this.endPose = endPose;
        this.controller = getController(constants);
        this.tolerance = getTolerance(constants);

        timer = new Timer();

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        Swerve.getInstance().resetModulesToAbsolute();
    }

    @Override
    public boolean isFinished() {
        Transform2d diff = Swerve.getInstance().getPose().minus(endPose);
        double xDiff = diff.getX();
        double yDiff = diff.getY();
        double rotDifDeg = diff.getRotation().getDegrees();

        ChassisSpeeds robotSpeeds = Swerve.getInstance().getRobotRelativeSpeeds();

        SmartDashboard.putBoolean("X Diff to End Pose", Math.abs(xDiff) <= tolerance.getX());
        SmartDashboard.putBoolean("Y Diff to End Pose", Math.abs(yDiff) <= tolerance.getY());
        SmartDashboard.putBoolean("Y Diff to End Pose", Math.abs(yDiff) <= tolerance.getY());
        SmartDashboard.putBoolean("Rot Diff to End Pose", Math.abs(rotDifDeg) <= tolerance.getRotation().getDegrees());
        SmartDashboard.putBoolean("X Vel", Math.abs(robotSpeeds.vxMetersPerSecond) <= constants.velXTolerance());
        SmartDashboard.putBoolean("Y Vel", Math.abs(robotSpeeds.vyMetersPerSecond) <= constants.velYTolerance());
        SmartDashboard.putBoolean("Rot Vel", Math.abs(robotSpeeds.omegaRadiansPerSecond) <= constants.rotVelTolerance());

        return Math.abs(xDiff) <= tolerance.getX()
                && Math.abs(yDiff) <= tolerance.getY()
                && Math.abs(rotDifDeg) <= tolerance.getRotation().getDegrees()
                && Math.abs(robotSpeeds.vxMetersPerSecond) <= constants.velXTolerance()
                && Math.abs(robotSpeeds.vyMetersPerSecond) <= constants.velYTolerance()
                && Math.abs(robotSpeeds.omegaRadiansPerSecond) <= constants.rotVelTolerance();
    }

    @Override
    public void end(boolean interrupted) {
        chassis.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        SmartDashboard.putString("Status", "Finished");
    }
}
