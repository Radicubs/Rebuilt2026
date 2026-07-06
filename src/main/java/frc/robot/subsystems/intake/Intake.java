package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private boolean pidEnabled = false;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;

    public static Intake getInstance(){
        if(INSTANCE == null) {INSTANCE = new Intake();}
        return INSTANCE;
    }

    private Intake(){

        io = RobotBase.isSimulation() ? new IntakeIOSim() : new IntakeIOReal();

        controller = new PIDController(IntakeConstants.PIDFeedforwardConstants.P, IntakeConstants.PIDFeedforwardConstants.I, IntakeConstants.PIDFeedforwardConstants.D);
        controller.setTolerance(IntakeConstants.PIDFeedforwardConstants.pidTolerance);

        feedforward = new SimpleMotorFeedforward(IntakeConstants.PIDFeedforwardConstants.S, IntakeConstants.PIDFeedforwardConstants.V, IntakeConstants.PIDFeedforwardConstants.A);
        IntakeLogger.publish(this);
    }

    public double getSetpoint() { return controller.getSetpoint(); }
    public double getVelocity() { return inputs.velocityRPS; }
    boolean atSetpoint() { return controller.atSetpoint(); }
    boolean isPidEnabled() { return pidEnabled; }
    public void setVelocity(double targetVelocity) {
        controller.reset();
        controller.setSetpoint(targetVelocity);
        pidEnabled = true;
    }
    public void cancelPID() {
        io.setDutyCycle(0);
        pidEnabled = false;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        IntakeLogger.log(this);

        if (pidEnabled) {
            double motorSpeed = controller.calculate(getVelocity());
            double ffValue = feedforward.calculate(getSetpoint());
            io.setDutyCycle(motorSpeed + ffValue);
        }
    }

}
