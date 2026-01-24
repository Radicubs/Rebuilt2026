package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

import java.util.Vector;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    DoubleSupplier translationX;
    DoubleSupplier translationY;
    DoubleSupplier rotation;
    BooleanSupplier alignToggle;

    private Swerve swerve;
    private PhotonVision photonVision;

    private double targetRobotAngle, rotSpeed;
    private boolean lockOn;

    public TeleopSwerve(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier alignToggle) {
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        this.alignToggle = alignToggle;
        swerve = Swerve.getInstance();
        photonVision = PhotonVision.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if(alignToggle.getAsBoolean())
            lockOn = true;

        if(rotation.getAsDouble() != 0)
            lockOn = false;

        if(lockOn){
            if(photonVision.getBestTag() != -1){
                swerve.setPose(photonVision.getRobotFieldPose());
            }

            targetRobotAngle = new Translation2d(
                    Constants.Swerve.hubPosition.getX() - swerve.getPose().getX(),
                    Constants.Swerve.hubPosition.getY() - swerve.getPose().getY()
            ).getAngle().getRadians();

            rotSpeed = (targetRobotAngle - swerve.getHeading().getRadians()) * Constants.Swerve.lockKP;
            if(rotSpeed < -1) rotSpeed = -1;
            if(rotSpeed > 1) rotSpeed = 1;
            if(Math.abs(rotSpeed) - Constants.Swerve.lockDeadband < 0) rotSpeed = 0;

            //REMOVE TO USE LOCK ON VALUES
            //rotSpeed = rotation.getAsDouble();
        }
        else{
            rotSpeed = rotation.getAsDouble();
        }


        SmartDashboard.putNumber("Target Rotation", new Translation2d(Constants.Swerve.hubPosition.getX() - swerve.getPose().getX(), Constants.Swerve.hubPosition.getY() - swerve.getPose().getY()).getAngle().getRadians());
        SmartDashboard.putNumber("Actual Rotation", swerve.getPose().getRotation().getRadians());
        SmartDashboard.putNumber("Last Tag", photonVision.getBestTag());
        SmartDashboard.putNumber("Rotation Speed", rotSpeed);

        swerve.drive(
                new Translation2d(
                        translationX.getAsDouble() * Constants.Swerve.maxSpeed,
                        translationY.getAsDouble() * Constants.Swerve.maxSpeed
                ),
                rotSpeed * Constants.Swerve.maxAngularVelocity,
                true,
                false);
    }
    public void initialize() {swerve.resetModulesToAbsolute();}
}