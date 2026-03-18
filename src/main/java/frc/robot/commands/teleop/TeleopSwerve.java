package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    DoubleSupplier translationX;
    DoubleSupplier translationY;
    DoubleSupplier rotation;
    BooleanSupplier toggleAlign;

    private Swerve swerve;
    private PhotonVision photonVision;

    private final double LOCKON_DEADBAND = 0.01;

    private double targetRobotAngle, rotSpeed;
    private boolean lockOn;
    private boolean prevState;

    public TeleopSwerve(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier toggleAlign) {
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        this.toggleAlign = toggleAlign;

        swerve = Swerve.getInstance();
        photonVision = PhotonVision.getInstance();
        prevState = false;
        addRequirements(swerve);
    }


    @Override
    public void execute() {
        if(!prevState && toggleAlign.getAsBoolean())
            lockOn = !lockOn;

        if(rotation.getAsDouble() != 0)
            lockOn = false;

        if(lockOn){
            double xTranslation = Constants.Field.hubCenter.getX() - swerve.getPose().getX();
            double yTranslation = Constants.Field.hubCenter.getY() - swerve.getPose().getY();
            int tagId = photonVision.getBestTag();
            if(tagId != -1){
                swerve.setPose(photonVision.getRobotFieldPose());
                Pose2d tagPose = photonVision.APRIL_TAG_LAYOUT.getTagPose(tagId).get().toPose2d();
                xTranslation = tagPose.getX() - swerve.getPose().getX();
                yTranslation = tagPose.getY() - swerve.getPose().getY();
            }

            targetRobotAngle = new Translation2d(
                    xTranslation,
                    yTranslation
            ).getAngle().getRadians();

            rotSpeed = (targetRobotAngle - swerve.getHeading().getRadians()) * Constants.Swerve.lockKP;
            if(rotSpeed < -Constants.Swerve.lockOnMaxSpeed) rotSpeed = -Constants.Swerve.lockOnMaxSpeed;
            if(rotSpeed > Constants.Swerve.lockOnMaxSpeed) rotSpeed = Constants.Swerve.lockOnMaxSpeed;
            if(Math.abs(rotSpeed) - Constants.Swerve.lockDeadband < 0) rotSpeed = 0;

            //REMOVE TO USE LOCK ON VALUES
            //rotSpeed = rotation.getAsDouble();
        }
        else{
            rotSpeed = rotation.getAsDouble();
        }

        SmartDashboard.putNumber("Last Tag", photonVision.getBestTag());
        Logger.recordOutput("Actual Speed Gyro", Swerve.getInstance().getGyroYaw());
        Logger.recordOutput("Actual Speed Heading", Swerve.getInstance().getHeading());

        swerve.drive(
                new Translation2d(
                        translationX.getAsDouble() * Constants.Swerve.maxSpeed,
                        translationY.getAsDouble() * Constants.Swerve.maxSpeed
                ),
                rotSpeed * Constants.Swerve.maxAngularVelocity,
                true,
                false);

        SmartDashboard.putBoolean("Lock on Active", lockOn);
        prevState = toggleAlign.getAsBoolean();
    }
    public void initialize() {swerve.resetModulesToAbsolute();}
}