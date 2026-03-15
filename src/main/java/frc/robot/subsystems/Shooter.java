package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private static Shooter INSTANCE;

    private TalonFXConfiguration indexerConfig;
    private TalonFXConfiguration topShooterConfig;

    private TalonFX indexer;
    private TalonFX topShooter;

    private TalonFXConfiguration leftConfig;
    private TalonFXConfiguration rightConfig;

    private TalonFX leftShooter;
    private TalonFX rightShooter;

    private final VelocityVoltage leftShooterVel = new VelocityVoltage(0), rightShooterVel = new VelocityVoltage(0), indexerVel = new VelocityVoltage(0), topShooterVel = new VelocityVoltage(0);


    private final SimpleMotorFeedforward indexerFF = new SimpleMotorFeedforward(Constants.Shooter.IndexerPIDFeedforwardConstants.kS, Constants.Shooter.IndexerPIDFeedforwardConstants.kV, Constants.Shooter.IndexerPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward topShooterFF = new SimpleMotorFeedforward(Constants.Shooter.TopShooterPIDFeedforwardConstants.kS, Constants.Shooter.TopShooterPIDFeedforwardConstants.kV, Constants.Shooter.TopShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward leftMainShooterFF = new SimpleMotorFeedforward(Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kS, Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kV, Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kA);
    private final SimpleMotorFeedforward rightMainShooterFF = new SimpleMotorFeedforward(Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kS, Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kV, Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kA);


    private boolean enablePID = false;

    public static Shooter getInstance() {
        if(INSTANCE == null) {INSTANCE = new Shooter();}
        return INSTANCE;
    }

    private Shooter(){
        // Indexer Config
        indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.indexerEnableCurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.indexerShooterCurrenLimit;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        indexerConfig.Slot0.kP = Constants.Shooter.IndexerPIDFeedforwardConstants.kP;
        indexerConfig.Slot0.kI = Constants.Shooter.IndexerPIDFeedforwardConstants.kI;
        indexerConfig.Slot0.kD = Constants.Shooter.IndexerPIDFeedforwardConstants.kD;


        // Top Shooter Config
        topShooterConfig = new TalonFXConfiguration();
        topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.topShooterEnableCurrentLimit;
        topShooterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.topShooterCurrentLimit;
        topShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        topShooterConfig.Slot0.kP = Constants.Shooter.TopShooterPIDFeedforwardConstants.kP;
        topShooterConfig.Slot0.kI = Constants.Shooter.TopShooterPIDFeedforwardConstants.kI;
        topShooterConfig.Slot0.kD = Constants.Shooter.TopShooterPIDFeedforwardConstants.kD;


        // Left Motor Config
        leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.shooterEnableCurrentLimit;
        leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shooterCurrentLimit;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: Change if needed

        leftConfig.Slot0.kP = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kP;
        leftConfig.Slot0.kI = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kI;
        leftConfig.Slot0.kD = Constants.Shooter.MainLeftShooterPIDFeedforwardConstants.kD;

        // Right Motor Config
        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.shooterEnableCurrentLimit;
        rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.shooterCurrentLimit;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: Change if needed

        rightConfig.Slot0.kP = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kP;
        rightConfig.Slot0.kI = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kI;
        rightConfig.Slot0.kD = Constants.Shooter.MainRightShooterPIDFeedforwardConstants.kD;

        // Indexer
        indexer = new TalonFX(Constants.Shooter.indexerCID);
        indexer.getConfigurator().apply(indexerConfig);

        // Top Shooter
        topShooter = new TalonFX(Constants.Shooter.topShooterCID);
        topShooter.getConfigurator().apply(topShooterConfig);

        // Left Shooter
        leftShooter = new TalonFX(Constants.Shooter.leftShooterCID);
        leftShooter.getConfigurator().apply(leftConfig);

        // Right Shooter
        rightShooter = new TalonFX(Constants.Shooter.rightShooterCID);
        rightShooter.getConfigurator().apply(rightConfig);
    }

    public double getIndexerSpeed(){
        return indexer.getVelocity().getValue().in(Units.RotationsPerSecond);
    }
    public double getTopShooterSpeed(){
        return topShooter.getVelocity().getValue().in(Units.RotationsPerSecond);
    }

    public void cancelPID(){
        leftShooter.set(0);
        rightShooter.set(0);

        indexer.set(0);

        topShooter.set(0);

        enablePID = false;
    }

    public void setShooterSpeeds(double mainShooterRPS, double topShaftRPS, double indexerRPS){

        indexerVel.Velocity = indexerRPS;
        indexerVel.FeedForward = indexerFF.calculate(indexerRPS);

        topShooterVel.Velocity = indexerRPS;
        topShooterVel.FeedForward = topShooterFF.calculate(indexerRPS);

        leftShooterVel.Velocity = mainShooterRPS;
        leftShooterVel.FeedForward = leftMainShooterFF.calculate(mainShooterRPS);

        rightShooterVel.Velocity = mainShooterRPS;
        rightShooterVel.FeedForward = rightMainShooterFF.calculate(mainShooterRPS);

        rightShooter.setControl(rightShooterVel);
        leftShooter.setControl(leftShooterVel);

        indexer.setControl(indexerVel);
        topShooter.setControl(topShooterVel);

        enablePID = true;
    }


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Left Shooter Speed", leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Shooter Speed", rightShooter.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Indexer speed", getIndexerSpeed());

        SmartDashboard.putNumber("Top Shooter", getTopShooterSpeed());

        SmartDashboard.putNumber("Left shooter current draw", leftShooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right shooter current draw", rightShooter.getSupplyCurrent().getValueAsDouble());


    }
}
