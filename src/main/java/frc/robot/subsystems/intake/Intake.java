package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE;
    private SparkMax intakeMotor;
    private boolean pidEnabled = false;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;
    private final RelativeEncoder encoder;

    public static Intake getInstance(){
        if(INSTANCE == null) {INSTANCE = new Intake();}
        return INSTANCE;
    }

    private Intake(){

        intakeMotor = new SparkMax(IntakeConstants.intakeMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(false);
        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = intakeMotor.getEncoder();
        controller = new PIDController(IntakeConstants.PIDFeedforwardConstants.P, IntakeConstants.PIDFeedforwardConstants.I, IntakeConstants.PIDFeedforwardConstants.D);
        controller.setTolerance(IntakeConstants.PIDFeedforwardConstants.pidTolerance);

        feedforward = new SimpleMotorFeedforward(IntakeConstants.PIDFeedforwardConstants.S, IntakeConstants.PIDFeedforwardConstants.V, IntakeConstants.PIDFeedforwardConstants.A);
        IntakeLogger.publish(this);
    }

    public double getSetpoint() { return controller.getSetpoint(); }
    public double getVelocity() { return encoder.getVelocity()/60; }
    public void setVelocity(double targetVelocity) {
        intakeMotor.set(0);
        pidEnabled = true;
        controller.setSetpoint(targetVelocity);
    }
    public void cancelPID() {
        intakeMotor.set(0);
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
