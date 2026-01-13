package frc.robot.commands.pathcommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

public class FollowTrajectory extends PathFollowingCommand {
    private final boolean followTrajectoryHeading;
    private final Trajectory trajectory;

    public FollowTrajectory(Trajectory trajectory, boolean followTrajectoryHeading, TrajectoryConstants constants) {
        super(Swerve.getInstance(), trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters, constants);

        this.followTrajectoryHeading = followTrajectoryHeading;
        this.trajectory = trajectory;
        addRequirements(Swerve.getInstance());
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.start();
        chassis.displayTrajectory(trajectory);
        SmartDashboard.putString("Status", "Running");
    }

    @Override
    public void execute() {
        if(timer.get() > 0.15 && timer.get() < 0.2) Swerve.getInstance().resetModulesToAbsolute();
        Trajectory.State state = trajectory.sample(timer.get());
        ChassisSpeeds speeds = controller.calculate(chassis.getPose(), state, followTrajectoryHeading ? state.poseMeters.getRotation() : endPose.getRotation());
        SmartDashboard.putNumber("PID Target X", controller.getXController().getSetpoint());
        SmartDashboard.putNumber("PID Target Y", controller.getYController().getSetpoint());
        SmartDashboard.putNumber("PID Target Angle", controller.getThetaController().getSetpoint().position);
        chassis.driveRobotRelative(speeds);
    }
}
