package frc.robot;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

import java.util.logging.Logger;

public class SwerveModule implements Sendable {


    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private SwerveModuleState desiredSwerveState;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final MotionMagicVoltage anglePositionM = new MotionMagicVoltage(0);
    private final PositionVoltage anglePositionP = new PositionVoltage(0);
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(moduleConstants.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(moduleConstants.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(moduleConstants.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setNeutralMode(NeutralModeValue.Brake);
        mDriveMotor.getConfigurator().setPosition(0.0);

        SmartDashboard.putData("Swerve Module " + moduleNumber, this);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");
        builder.addDoubleProperty("CANCoder", () -> getCANcoder().getDegrees(), null);
        builder.addDoubleProperty("Velocity", () -> getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Angle", () -> getState().angle.getDegrees(), null);
        builder.addDoubleProperty("Desired Velocity", () -> (desiredSwerveState!=null) ? desiredSwerveState.speedMetersPerSecond : 0, null);
        builder.addDoubleProperty("Module " + moduleNumber + " current - Drive Motor", ()  -> mDriveMotor.getSupplyCurrent().getValue().in(Units.Amp), null);
        builder.addDoubleProperty("Module " + moduleNumber + " current - Angle Motor", ()  -> mAngleMotor.getSupplyCurrent().getValue().in(Units.Amp), null);

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        //desiredState.optimize(getState().angle);
        desiredSwerveState = desiredState;
        if (Constants.Swerve.useMagicMotion) {
            mAngleMotor.setControl(anglePositionM.withPosition(desiredState.angle.getRotations()));
        } else {
            mAngleMotor.setControl(anglePositionP.withPosition(desiredState.angle.getRotations()));
        }
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            double driveVel = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            if(driveVelocity.Velocity != driveVel){
                driveVelocity.Velocity = driveVel;
                driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
                mDriveMotor.setControl(driveVelocity);
            }
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue().in(Units.Rotation));
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue().in(Units.RotationsPerSecond), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue().in(Units.Rotation))
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue().in(Units.Rotation), Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue().in(Units.Rotation))
        );
    }
}