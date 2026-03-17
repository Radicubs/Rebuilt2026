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
        pivotMotorConfig.inverted(false); //TODO: CHANGE IF NEEDED
        pivotMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotMotorConfig.encoder.positionConversionFactor(1.0/135);
        pivotMotorConfig.encoder.velocityConversionFactor(1.0/135);
        pivotMotorConfig.smartCurrentLimit(Constants.Pivot.pivotMotorStallCurrentLimit, Constants.Pivot.pivotMotorFreeCurrentLimit);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        relativeEncoder = pivotMotor.getEncoder();
        relativeEncoder.setPosition(Constants.Pivot.upPos);

        pid = new ProfiledPIDController(Constants.Pivot.PIDFeedforwardConstants.P, Constants.Pivot.PIDFeedforwardConstants.I, Constants.Pivot.PIDFeedforwardConstants.D, new TrapezoidProfile.Constraints(1000, 1000));
        pid.setTolerance(0.01);

        feedforward = new ArmFeedforward(Constants.Pivot.PIDFeedforwardConstants.S, Constants.Pivot.PIDFeedforwardConstants.G, Constants.Pivot.PIDFeedforwardConstants.V);

        SmartDashboard.putData("Pivot", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Pivot Angle",() -> getPosition(), null);
                builder.addDoubleProperty("Pivot Motor RPS", relativeEncoder::getVelocity, null);
                builder.addDoubleProperty("Pivot Motor Desired Angle", () -> getGoal(), null);
                builder.addDoubleProperty("Pivot Motor current draw", () -> pivotMotor.getOutputCurrent(), null);
                builder.addBooleanProperty("Pivot Extended", () -> (getPosition() >= Constants.Pivot.downPos - 0.1) && (getPosition() <= Constants.Pivot.downPos + 0.1), null);

            }
        });
    }
    public double getPosition() {
        return relativeEncoder.getPosition();
    }
    public double getGoal() {
        return pid.getGoal().position;
    }
    public void setGoal(double targetRotation){
        reset();
        moveToTargetAngle = true;
        pid.setGoal(new TrapezoidProfile.State(targetRotation, Constants.Pivot.pivotFinalVelocity));
    }
    public void reset() {
        pivotMotor.set(0);
    }
    public void resetAngle(){
        relativeEncoder.setPosition(Constants.Pivot.downPos);
    }


    public void setSpeed(double speed){
        pivotMotor.set(speed);
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
        }
    }
}