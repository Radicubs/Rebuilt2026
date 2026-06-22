package frc.robot.subsystems.transfer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {

    private static Transfer INSTANCE;
    private SparkMax beltMotor;
    private final PIDController transferController;
    private final SimpleMotorFeedforward transferFeedforward;
    private boolean goToTransferTarget = false;

    public static Transfer getInstance(){
        if(INSTANCE == null) {INSTANCE = new Transfer();}
        return INSTANCE;
    }

    private Transfer(){

        beltMotor = new SparkMax(TransferConstants.beltMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
        beltMotorConfig.inverted(false);
        beltMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        beltMotorConfig.smartCurrentLimit(TransferConstants.beltMotorStallCurrentLimit, TransferConstants.beltMotorFreeCurrentLimit);
        beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        transferController = new PIDController(TransferConstants.TransferPIDFeedforwardConstants.kP, TransferConstants.TransferPIDFeedforwardConstants.kI, TransferConstants.TransferPIDFeedforwardConstants.kD);
        transferFeedforward = new SimpleMotorFeedforward(TransferConstants.TransferPIDFeedforwardConstants.kS, TransferConstants.TransferPIDFeedforwardConstants.kV, TransferConstants.TransferPIDFeedforwardConstants.kA);

        TransferLogger.publish(this);

    }

    public double getTransferSpeed(){
        return beltMotor.getEncoder().getVelocity()/60.0;
    }
    public void setTransferSpeed(double targetSpeed){
        beltMotor.set(0);
        goToTransferTarget = true;
        transferController.setSetpoint(targetSpeed);
    }
    public void cancelPID(){
        beltMotor.set(0);
        goToTransferTarget=false;
    }

    @Override
    public void periodic() {
        if(goToTransferTarget){
            double transferMotorSpeed = transferController.calculate(getTransferSpeed());
            double transferFeedForwardVal = transferFeedforward.calculate(transferController.getSetpoint());

            beltMotor.set(transferMotorSpeed + transferFeedForwardVal);

        }
    }

}
