package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
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

    private PIDController lockOnPID;

    public TeleopSwerve(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier toggleAlign) {
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        this.toggleAlign = toggleAlign;


        lockOnPID = new PIDController(Constants.Swerve.lockKP, 0, 0);
        lockOnPID.enableContinuousInput(-Math.PI, Math.PI);

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

        Pose2d curPose = photonVision.getRobotFieldPose();
        if(lockOn && DriverStation.getAlliance().isPresent()){
            double xTranslation;
            double yTranslation;

            if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
                xTranslation = Constants.Field.hubCenterBlue.getX() - curPose.getX();
                yTranslation = Constants.Field.hubCenterBlue.getY() - curPose.getY();
            }
            else{
                xTranslation = Constants.Field.hubCenterRed.getX() - curPose.getX();
                yTranslation = Constants.Field.hubCenterRed.getY() - curPose.getY();
            }

            targetRobotAngle = new Translation2d(
                    xTranslation,
                    yTranslation
            ).getAngle().plus(Rotation2d.k180deg).getRadians();

            rotSpeed = lockOnPID.calculate(photonVision.visionOdometry.getPoseMeters().getRotation().getRadians(), targetRobotAngle);
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
                        translationY.getAsDouble()* Constants.Swerve.maxSpeed
                ),
                rotSpeed * Constants.Swerve.maxAngularVelocity,
                true,
                false);

        SmartDashboard.putBoolean("Lock on Active", lockOn);
        prevState = toggleAlign.getAsBoolean();
    }
    public void initialize() {swerve.resetModulesToAbsolute();}
}