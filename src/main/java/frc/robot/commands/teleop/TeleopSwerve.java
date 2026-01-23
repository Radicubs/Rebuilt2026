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
    PIDController rotController;
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
        rotController = new PIDController(0.1, 0, 0); //TODO: FIX ERROR CALCULATION
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

            rotController.setSetpoint(targetRobotAngle);

            rotSpeed = rotController.calculate(swerve.getHeading().getRadians());
            if(rotSpeed < -1) rotSpeed = -1;
            if(rotSpeed > 1) rotSpeed = 1;

            //REMOVE TO USE LOCK ON VALUES
            rotSpeed = rotation.getAsDouble();
        }
        else{
            rotSpeed = rotation.getAsDouble();
        }


        SmartDashboard.putNumber("Target Rotation", new Translation2d(Constants.Swerve.hubPosition.getX() - swerve.getPose().getX(), Constants.Swerve.hubPosition.getY() - swerve.getPose().getY()).getAngle().getDegrees());
        SmartDashboard.putNumber("Actual Rotation", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Last Tag", photonVision.getBestTag());
        SmartDashboard.putNumber("PID", rotController.calculate(swerve.getHeading().getDegrees()));

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