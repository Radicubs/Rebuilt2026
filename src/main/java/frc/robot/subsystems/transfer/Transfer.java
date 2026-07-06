package frc.robot.subsystems.transfer;

import frc.robot.constants.TransferConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {

    private static Transfer INSTANCE;
    private final TransferIO io;
    private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();
    private final PIDController transferController;
    private final SimpleMotorFeedforward transferFeedforward;
    private boolean goToTransferTarget = false;

    public static Transfer getInstance(){
        if(INSTANCE == null) {INSTANCE = new Transfer();}
        return INSTANCE;
    }

    private Transfer(){

        io = RobotBase.isSimulation() ? new TransferIOSim() : new TransferIOReal();

        transferController = new PIDController(TransferConstants.TransferPIDFeedforwardConstants.kP, TransferConstants.TransferPIDFeedforwardConstants.kI, TransferConstants.TransferPIDFeedforwardConstants.kD);
        transferFeedforward = new SimpleMotorFeedforward(TransferConstants.TransferPIDFeedforwardConstants.kS, TransferConstants.TransferPIDFeedforwardConstants.kV, TransferConstants.TransferPIDFeedforwardConstants.kA);

        TransferLogger.publish(this);

    }

    public double getTransferSpeed(){
        return inputs.velocityRPS;
    }
    double getSetpoint(){ return transferController.getSetpoint(); }
    boolean isActive(){ return goToTransferTarget; }
    public void setTransferSpeed(double targetSpeed){
        transferController.reset();
        transferController.setSetpoint(targetSpeed);
        goToTransferTarget = true;
    }
    public void cancelPID(){
        io.setDutyCycle(0);
        goToTransferTarget=false;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);
        TransferLogger.log(this);
        if(goToTransferTarget){
            double transferMotorSpeed = transferController.calculate(getTransferSpeed());
            double transferFeedForwardVal = transferFeedforward.calculate(transferController.getSetpoint());
            io.setDutyCycle(transferMotorSpeed + transferFeedForwardVal);
        }
        else{
            io.setDutyCycle(0);
        }
    }

}
