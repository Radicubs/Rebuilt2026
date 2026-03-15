package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE;

    private SparkMax intakeMotor;

    public double targetVelocity;
    private boolean pidEnabled = false;

    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;
    private final RelativeEncoder encoder;

    public static Intake getInstance(){
        if(INSTANCE == null) {INSTANCE = new Intake();}
        return INSTANCE;
    }

    private Intake(){
        intakeMotor = new SparkMax(Constants.Intake.intakeMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(false);
        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = intakeMotor.getEncoder();
        controller = new PIDController(Constants.Intake.PIDFeedforwardConstants.P, Constants.Intake.PIDFeedforwardConstants.I, Constants.Intake.PIDFeedforwardConstants.D);
        controller.setTolerance(0.05);

        feedforward = new SimpleMotorFeedforward(Constants.Intake.PIDFeedforwardConstants.S, Constants.Intake.PIDFeedforwardConstants.V, Constants.Intake.PIDFeedforwardConstants.A);
    }

    public void setIntakeSpeed(double RPS){
        setSetpoint(RPS);
    }
    public void reset() { intakeMotor.set(0); }
    public double getVelocity() { return encoder.getVelocity()/60; }
    public double getSetpoint() { return controller.getSetpoint(); }
    private void setSetpoint(double targetVelocity) {
        reset();
        pidEnabled = true;
        this.targetVelocity = targetVelocity;
        controller.setSetpoint(targetVelocity);
    }
    public void cancelPID() {
        pidEnabled = false;
    }

    @Override
    public void periodic() {
        if (pidEnabled) {
            double motorSpeed = controller.calculate(getVelocity());
            double ffValue = feedforward.calculate(getSetpoint());
            intakeMotor.set(motorSpeed + ffValue);
        }
        else{
            intakeMotor.set(0);
        }
    }
}
