package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

    private static Pivot INSTANCE;

    private SparkMax pivotMotor;

    private final ProfiledPIDController pid;
    private final ArmFeedforward feedforward;
    private final RelativeEncoder relativeEncoder;

    private boolean moveToTargetAngle = false;

    public static Pivot getInstance(){
        if(INSTANCE == null) {INSTANCE = new Pivot();}
        return INSTANCE;
    }

    private Pivot(){
        pivotMotor = new SparkMax(Constants.Pivot.pivotMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.inverted(false);
        pivotMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotMotorConfig.encoder.positionConversionFactor(1.0/60);
        pivotMotorConfig.encoder.velocityConversionFactor(1.0/60);
        pivotMotorConfig.smartCurrentLimit(Constants.Pivot.pivotMotorStallCurrentLimit, Constants.Pivot.pivotMotorFreeCurrentLimit);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        relativeEncoder = pivotMotor.getEncoder();
        relativeEncoder.setPosition(Constants.Pivot.upPos);

        pid = new ProfiledPIDController(Constants.Pivot.PIDFeedforwardConstants.P, Constants.Pivot.PIDFeedforwardConstants.I, Constants.Pivot.PIDFeedforwardConstants.D, new TrapezoidProfile.Constraints(.5, 3));
        pid.setTolerance(0.01);

        feedforward = new ArmFeedforward(Constants.Pivot.PIDFeedforwardConstants.S, Constants.Pivot.PIDFeedforwardConstants.G, Constants.Pivot.PIDFeedforwardConstants.V);

        SmartDashboard.putData("Pivot", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Desired Pivot Angle", () -> pid.getSetpoint().position, null);
                builder.addDoubleProperty("Current Pivot Angle", () -> getPosition(), null);
            }
        });
    }
    public double getPosition() {
        return relativeEncoder.getPosition();
    }
    public void setGoal(double targetRotation){
        reset();
        cancelPID();
        moveToTargetAngle = true;
        pid.setGoal(new TrapezoidProfile.State(targetRotation, Constants.Pivot.pivotFinalVelocity));
    }
    public void reset() {
        pivotMotor.set(0);
    }
    public void resetAngle(){
        relativeEncoder.setPosition(Constants.Pivot.downPos);
    }

    public double getSpeed(){
        return pivotMotor.get();
    }

    public void setSpeed(double speed){
        moveToTargetAngle = false;
        pivotMotor.set(speed);
    }
    public boolean atGoal(){
        return pid.atGoal();
    }

    public void cancelPID(){
        moveToTargetAngle = false;
        pivotMotor.set(0);
        pid.reset(relativeEncoder.getPosition());
    }

    @Override
    public void periodic() {
        if (moveToTargetAngle) {
            double motorSpeed = pid.calculate(getPosition());
            double feedforwardVal = feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);

            pivotMotor.set(motorSpeed + feedforwardVal);

            if(pid.atGoal()){cancelPID();}

        }
    }
}