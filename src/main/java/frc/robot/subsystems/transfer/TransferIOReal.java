package frc.robot.subsystems.transfer;

import frc.robot.constants.TransferConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

class TransferIOReal implements TransferIO {

    private final SparkMax beltMotor;
    private final RelativeEncoder encoder;

    TransferIOReal() {
        SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
        beltMotorConfig.inverted(false);
        beltMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        beltMotorConfig.smartCurrentLimit(TransferConstants.beltMotorStallCurrentLimit, TransferConstants.beltMotorFreeCurrentLimit);

        beltMotor = new SparkMax(TransferConstants.beltMotorCID, SparkLowLevel.MotorType.kBrushless);
        beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = beltMotor.getEncoder();
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        inputs.velocityRPS = encoder.getVelocity() / 60.0;
        inputs.appliedVolts = beltMotor.getAppliedOutput() * beltMotor.getBusVoltage();
        inputs.currentAmps = beltMotor.getOutputCurrent();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        beltMotor.set(dutyCycle);
    }
}
