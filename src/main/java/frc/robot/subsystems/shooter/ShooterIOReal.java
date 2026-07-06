package frc.robot.subsystems.shooter;

import frc.robot.constants.ShooterConstants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class ShooterIOReal implements ShooterIO {

    private final TalonFX leftShooter;
    private final TalonFX rightShooter;
    private final TalonFX topShooter;
    private final TalonFX indexer;

    private final VelocityVoltage leftVel = new VelocityVoltage(0), rightVel = new VelocityVoltage(0), topVel = new VelocityVoltage(0), indexerVel = new VelocityVoltage(0);


    private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kS,
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kV,
            ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kS,
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kV,
            ShooterConstants.MainRightShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward topFF = new SimpleMotorFeedforward(
            ShooterConstants.TopShooterPIDFeedforwardConstants.kS,
            ShooterConstants.TopShooterPIDFeedforwardConstants.kV,
            ShooterConstants.TopShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward indexerFF = new SimpleMotorFeedforward(
            ShooterConstants.IndexerPIDFeedforwardConstants.kS,
            ShooterConstants.IndexerPIDFeedforwardConstants.kV,
            ShooterConstants.IndexerPIDFeedforwardConstants.kA);

    ShooterIOReal() {
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.indexerEnableCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.indexerShooterCurrentLimit;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed
        indexerConfig.Slot0.kP = ShooterConstants.IndexerPIDFeedforwardConstants.kP;
        indexerConfig.Slot0.kI = ShooterConstants.IndexerPIDFeedforwardConstants.kI;
        indexerConfig.Slot0.kD = ShooterConstants.IndexerPIDFeedforwardConstants.kD;

        TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();
        topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.topShooterEnableCurrentLimit;
        topShooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.topShooterCurrentLimit;
        topShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed
        topShooterConfig.Slot0.kP = ShooterConstants.TopShooterPIDFeedforwardConstants.kP;
        topShooterConfig.Slot0.kI = ShooterConstants.TopShooterPIDFeedforwardConstants.kI;
        topShooterConfig.Slot0.kD = ShooterConstants.TopShooterPIDFeedforwardConstants.kD;

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterEnableCurrentLimit;
        leftConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed
        leftConfig.Slot0.kP = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kP;
        leftConfig.Slot0.kI = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kI;
        leftConfig.Slot0.kD = ShooterConstants.MainLeftShooterPIDFeedforwardConstants.kD;

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.shooterEnableCurrentLimit;
        rightConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterCurrentLimit;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: Change if needed
        rightConfig.Slot0.kP = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kP;
        rightConfig.Slot0.kI = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kI;
        rightConfig.Slot0.kD = ShooterConstants.MainRightShooterPIDFeedforwardConstants.kD;

        indexer = new TalonFX(ShooterConstants.indexerCID);
        indexer.getConfigurator().apply(indexerConfig);

        topShooter = new TalonFX(ShooterConstants.topShooterCID);
        topShooter.getConfigurator().apply(topShooterConfig);

        leftShooter = new TalonFX(ShooterConstants.leftShooterCID);
        leftShooter.getConfigurator().apply(leftConfig);

        rightShooter = new TalonFX(ShooterConstants.rightShooterCID);
        rightShooter.getConfigurator().apply(rightConfig);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftVelocityRps = leftShooter.getVelocity().getValueAsDouble();
        inputs.rightVelocityRps = rightShooter.getVelocity().getValueAsDouble();
        inputs.topVelocityRps = topShooter.getVelocity().getValueAsDouble();
        inputs.indexerVelocityRps = indexer.getVelocity().getValueAsDouble();

        inputs.leftAppliedVolts = leftShooter.getMotorVoltage().getValueAsDouble();
        inputs.rightAppliedVolts = rightShooter.getMotorVoltage().getValueAsDouble();
        inputs.topAppliedVolts = topShooter.getMotorVoltage().getValueAsDouble();
        inputs.indexerAppliedVolts = indexer.getMotorVoltage().getValueAsDouble();

        inputs.leftCurrentAmps = leftShooter.getStatorCurrent().getValueAsDouble();
        inputs.rightCurrentAmps = rightShooter.getStatorCurrent().getValueAsDouble();
        inputs.topCurrentAmps = topShooter.getStatorCurrent().getValueAsDouble();
        inputs.indexerCurrentAmps = indexer.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setShooterSpeeds(double mainRps, double topRps, double indexerRps) {
        leftShooter.setControl(leftVel.withVelocity(mainRps).withFeedForward(leftFF.calculate(mainRps)));
        rightShooter.setControl(rightVel.withVelocity(mainRps).withFeedForward(rightFF.calculate(mainRps)));
        topShooter.setControl(topVel.withVelocity(topRps).withFeedForward(topFF.calculate(topRps)));
        indexer.setControl(indexerVel.withVelocity(indexerRps).withFeedForward(indexerFF.calculate(indexerRps)));
    }

    @Override
    public void setIndexerSpeed(double indexerRps) {
        indexer.setControl(indexerVel.withVelocity(indexerRps).withFeedForward(indexerFF.calculate(indexerRps)));
    }

    @Override
    public void stop() {
        leftShooter.set(0);
        rightShooter.set(0);
        topShooter.set(0);
        indexer.set(0);
    }
}
