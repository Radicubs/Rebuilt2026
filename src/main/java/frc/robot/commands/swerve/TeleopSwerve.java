package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.field.FieldConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    DoubleSupplier translationX;
    DoubleSupplier translationY;
    DoubleSupplier rotation;
    BooleanSupplier toggleAlign;

    private Swerve swerve;
    private PhotonVision photonVision;

    private double targetRobotAngle, rotSpeed;
    private boolean lockOn;
    private boolean prevState;

    private PIDController lockOnPID;

    public TeleopSwerve(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier toggleAlign) {
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        this.toggleAlign = toggleAlign;


        lockOnPID = new PIDController(SwerveConstants.lockKP, 0, 0);
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

        Pose2d curPose = Swerve.getInstance().getPose();
        if(lockOn && DriverStation.getAlliance().isPresent()){
            double xTranslation;
            double yTranslation;

            if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)){
                xTranslation = FieldConstants.hubCenterBlue.getX() - curPose.getX();
                yTranslation = FieldConstants.hubCenterBlue.getY() - curPose.getY();
            }
            else{
                xTranslation = FieldConstants.hubCenterRed.getX() - curPose.getX();
                yTranslation = FieldConstants.hubCenterRed.getY() - curPose.getY();
            }

            targetRobotAngle = new Translation2d(
                    xTranslation,
                    yTranslation
            ).getAngle().plus(Rotation2d.k180deg).getRadians();

            rotSpeed = lockOnPID.calculate(Swerve.getInstance().getPose().getRotation().getRadians(), targetRobotAngle);
            if(rotSpeed < -SwerveConstants.lockOnMaxSpeed) rotSpeed = -SwerveConstants.lockOnMaxSpeed;
            if(rotSpeed > SwerveConstants.lockOnMaxSpeed) rotSpeed = SwerveConstants.lockOnMaxSpeed;
            if(Math.abs(rotSpeed) - SwerveConstants.lockDeadband < 0) rotSpeed = 0;
        }
        else{
            rotSpeed = rotation.getAsDouble();
        }

        SmartDashboard.putNumber("Last Tag", photonVision.getBestTag());

        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        swerve.drive(
                (!isRed) ?
                    new Translation2d(
                            translationX.getAsDouble() * SwerveConstants.maxSpeed,
                            translationY.getAsDouble()* SwerveConstants.maxSpeed
                    ) :
                    new Translation2d(
                            -translationX.getAsDouble() * SwerveConstants.maxSpeed,
                            -translationY.getAsDouble()* SwerveConstants.maxSpeed
                )
                ,
                rotSpeed * SwerveConstants.maxAngularVelocity,
                true,
                false);

        SmartDashboard.putBoolean("Lock on Active", lockOn);
        prevState = toggleAlign.getAsBoolean();
    }
    public void initialize() {
        swerve.resetModulesToAbsolute();
    }
}