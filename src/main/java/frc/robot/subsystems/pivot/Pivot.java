package frc.robot.subsystems.pivot;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

        pivotMotor = new SparkMax(PivotConstants.pivotMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.inverted(false);
        pivotMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotMotorConfig.encoder.positionConversionFactor(1.0/60);
        pivotMotorConfig.encoder.velocityConversionFactor(1.0/60);
        pivotMotorConfig.smartCurrentLimit(PivotConstants.pivotMotorStallCurrentLimit, PivotConstants.pivotMotorFreeCurrentLimit);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        relativeEncoder = pivotMotor.getEncoder();
        relativeEncoder.setPosition(PivotConstants.upPos);

        pid = new ProfiledPIDController(PivotConstants.PIDFeedforwardConstants.P, PivotConstants.PIDFeedforwardConstants.I, PivotConstants.PIDFeedforwardConstants.D, new TrapezoidProfile.Constraints(.5, 3));
        pid.setTolerance(PivotConstants.PIDFeedforwardConstants.pidTolerance);

        feedforward = new ArmFeedforward(PivotConstants.PIDFeedforwardConstants.S, PivotConstants.PIDFeedforwardConstants.G, PivotConstants.PIDFeedforwardConstants.V);

        PivotLogger.publish(this);

    }
    public double getPosition() {
        return relativeEncoder.getPosition();
    }
    public double getSpeed(){
        return pivotMotor.get();
    }
    public void setSpeed(double speed){
        moveToTargetAngle = false;
        pivotMotor.set(speed);
    }
    public void setGoal(double targetRotation){
        cancelPID();
        moveToTargetAngle = true;
        pid.setGoal(new TrapezoidProfile.State(targetRotation, PivotConstants.pivotFinalVelocity));
    }
    public double getDesiredAngle() {
        return pid.getSetpoint().position;
    }
    public void resetAngle(){
        relativeEncoder.setPosition(PivotConstants.downPos);
    }
    public void cancelPID(){
        pivotMotor.set(0);
        moveToTargetAngle = false;
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