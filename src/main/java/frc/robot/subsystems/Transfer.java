package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        beltMotor = new SparkMax(Constants.Transfer.beltMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
        beltMotorConfig.inverted(false);
        beltMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        beltMotorConfig.smartCurrentLimit(Constants.Transfer.beltMotorStallCurrentLimit, Constants.Transfer.beltMotorFreeCurrentLimit);
        beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        transferController = new PIDController(Constants.Transfer.TransferPIDFeedforwardConstants.kP, Constants.Transfer.TransferPIDFeedforwardConstants.kI, Constants.Transfer.TransferPIDFeedforwardConstants.kD);
        transferFeedforward = new SimpleMotorFeedforward(Constants.Transfer.TransferPIDFeedforwardConstants.kS, Constants.Transfer.TransferPIDFeedforwardConstants.kV, Constants.Transfer.TransferPIDFeedforwardConstants.kA);
        SmartDashboard.putData("Transfer", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Transfer Speed",() -> getTransferSpeed(), null);
                builder.addDoubleProperty("Desired Transfer Speed", transferController::getSetpoint, null);
                builder.addDoubleProperty("Transfer Current Draw", () -> beltMotor.getOutputCurrent(), null);
            }
        });
    }

    public void reset(){
        beltMotor.set(0);
    }
    public void setTransferSetpoint(double transferTarget){
        reset();
        goToTransferTarget = true;
        transferController.setSetpoint(transferTarget);
    }

    public double getTransferSpeed(){
        return beltMotor.getEncoder().getVelocity()/60.0;
    }
    public void setTransferSpeed(double speed){
        setTransferSetpoint(speed);
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
