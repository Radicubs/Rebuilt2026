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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

    private static Pivot INSTANCE;

    private SparkMax pivotMotor;

    private final PIDController pid;
    private final ArmFeedforward feedforward;
    private final RelativeEncoder relativeEncoder;

    private boolean moveToTargetAngle = false;
    private double targetRotation = 0.0;

    public static Pivot getInstance(){
        if(INSTANCE == null) {INSTANCE = new Pivot();}
        return INSTANCE;
    }

    private Pivot(){
        pivotMotor = new SparkMax(Constants.Pivot.pivotMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.inverted(false); //TODO: CHANGE IF NEEDED
        pivotMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotMotorConfig.encoder.positionConversionFactor(1.0/45);
        pivotMotorConfig.encoder.velocityConversionFactor(1.0/45);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        relativeEncoder = pivotMotor.getEncoder();

        pid = new PIDController(Constants.Pivot.PIDFeedfowardConstants.P, Constants.Pivot.PIDFeedfowardConstants.I, Constants.Pivot.PIDFeedfowardConstants.D);
        pid.setTolerance(0.05);

        feedforward = new ArmFeedforward(Constants.Pivot.PIDFeedfowardConstants.S, Constants.Pivot.PIDFeedfowardConstants.G, Constants.Pivot.PIDFeedfowardConstants.V);
    }
    public double getPosition() {
        return relativeEncoder.getPosition();
    }
    public double getSetpoint() {
        return pid.getSetpoint();
    }
    public void setSetpoint(double targetRotation){
        reset();
        moveToTargetAngle = true;
        this.targetRotation = targetRotation;
        pid.setSetpoint(targetRotation);
    }
    public void reset() {
        pivotMotor.set(0);
    }
    public void zero() {
        relativeEncoder.setPosition(0);
    }

    public void cancelPID(){
        moveToTargetAngle = false;
    }

    public void setSpeed(double speed){
        pivotMotor.set(speed);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Real Position", getPosition());
        SmartDashboard.putNumber("Set Position", getSetpoint());

        if (moveToTargetAngle) {
            double motorSpeed = pid.calculate(getPosition());
            double feedforwardVal = feedforward.calculate(relativeEncoder.getPosition(), relativeEncoder.getVelocity());
            pivotMotor.set(motorSpeed - feedforwardVal);
        }
    }
}