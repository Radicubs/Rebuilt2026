package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

class IntakeIOReal implements IntakeIO {

    private final SparkMax intakeMotor;
    private final RelativeEncoder encoder;

    IntakeIOReal() {
        intakeMotor = new SparkMax(IntakeConstants.intakeMotorCID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(false);
        intakeConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = intakeMotor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRPS = encoder.getVelocity() / 60.0;
        inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        intakeMotor.set(dutyCycle);
    }
}
