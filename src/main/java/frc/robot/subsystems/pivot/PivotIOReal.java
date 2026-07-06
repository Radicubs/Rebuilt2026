package frc.robot.subsystems.pivot;

import frc.robot.constants.PivotConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

class PivotIOReal implements PivotIO {

    private final SparkMax pivotMotor;
    private final RelativeEncoder encoder;

    PivotIOReal() {
        pivotMotor = new SparkMax(PivotConstants.pivotMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        pivotMotorConfig.inverted(false);
        pivotMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotMotorConfig.encoder.positionConversionFactor(1.0 / 60);
        pivotMotorConfig.encoder.velocityConversionFactor(1.0 / 60);
        pivotMotorConfig.smartCurrentLimit(PivotConstants.pivotMotorStallCurrentLimit, PivotConstants.pivotMotorFreeCurrentLimit);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = pivotMotor.getEncoder();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRot = encoder.getPosition();
        inputs.appliedDuty = pivotMotor.get();
        inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.currentAmps = pivotMotor.getOutputCurrent();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        pivotMotor.set(dutyCycle);
    }

    @Override
    public void setPosition(double positionRot) {
        encoder.setPosition(positionRot);
    }
}
